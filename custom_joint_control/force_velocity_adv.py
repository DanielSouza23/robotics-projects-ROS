import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from roboticstoolbox import DHRobot, RevoluteDH
from scipy.spatial.transform import Rotation as R

# =============================
# Robot kinematics & cost functions
# =============================
xarm7 = DHRobot([
    RevoluteDH(d=0.267,  a=0,       alpha=-np.pi/2),
    RevoluteDH(d=0,      a=0,       alpha= np.pi/2),
    RevoluteDH(d=0.293,  a=0.0525,  alpha= np.pi/2),
    RevoluteDH(d=0,      a=-0.0775, alpha= np.pi/2),
    RevoluteDH(d=0.3425, a=0,       alpha= np.pi/2),
    RevoluteDH(d=0,      a=0.076,   alpha=-np.pi/2),
    RevoluteDH(d=0.197,  a=0,       alpha= 0)
], name='xArm7')


def forward_kinematics(q):
    T = xarm7.fkine(q)
    return T.t, T.R, T.A


def compute_jacobian(q):
    J = xarm7.jacob0(q)
    return J, J[3:6, :]


def cost_gradient_KMI(q):
    grad = np.zeros(7)
    eps = 1e-6
    J, _ = compute_jacobian(q)
    JJt = J @ J.T
    detJ = np.linalg.det(JJt)
    invJJt = np.linalg.pinv(JJt)
    w = np.sqrt(detJ) if detJ > 0 else 0.0
    for i in range(7):
        dq = np.zeros(7); dq[i] = eps
        Jp, _ = compute_jacobian(q + dq)
        Jm, _ = compute_jacobian(q - dq)
        dJ = (Jp - Jm) / (2 * eps)
        dJJt = dJ @ J.T + J @ dJ.T
        grad[i] = 0.5 * w * np.trace(invJJt @ dJJt) if w > 0 else 0.0
    return grad


def cost_gradient_JLA(q):
    q_min = np.deg2rad([-360, -118, -360, -11, -360, -97, -360])
    q_max = np.deg2rad([360, 120, 360, 225, 360, 180, 360])
    q_bar = (q_max + q_min) / 2
    q_t = 2 * (q - q_bar)
    q_b = (q_max - q_min) ** 2
    grad = q_t / q_b
    return -grad


def wrap_to_pi(angle):
    return (angle + np.pi) % (2*np.pi) - np.pi


# =============================
# Resolved Rates Controller (with pre-flip on x<0)
# =============================
class XArmResolvedRate(Node):
    def __init__(self):
        super().__init__('xarm_resolved_rates')
        self.start_time = None
        self.declare_parameter('rate', 100.0)
        self.rate = self.get_parameter('rate').value

        self.goal_reached = False
        self.q = np.zeros(7)
        self.q_initialized = False

        self.target_pose = None
        self.tool_flip = R.from_euler('x', 180, degrees=True)
        self.target_ori = R.from_euler('xyz', [0, 0, 0])  # placeholder

        # Base pre-flip state
        self.flip_required = False
        self.q1_target = 0.0
        self.base_flip_gain = 1.5         # rad/s per rad error
        self.base_flip_max_speed = 2    # rad/s cap
        self.base_flip_tol = 0.05         # rad

        # Control gains
        self.kp_x, self.kp_r, self.max_vel = 1, 0.5, 2

        # ROS
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        self.create_subscription(PoseStamped, '/target_pose', self.on_target, 10)
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.vel_pub = self.create_publisher(Float64MultiArray, '/xarm_velocity_controller/commands', 10)

        self.timer = self.create_timer(1 / self.rate, self.control_loop)
        self.control = 'Idle'

    def joint_state_callback(self, msg):
        if not self.q_initialized:
            self.q = np.array(msg.position[:7])
            self.q_initialized = True

    def on_target(self, msg):
        p = msg.pose.position
        self.start_time = self.get_clock().now()
        self.goal_reached = False
        self.target_pose = np.array([p.x, p.y, p.z])

        tgt = R.from_quat([msg.pose.orientation.x,
                           msg.pose.orientation.y,
                           msg.pose.orientation.z,
                           msg.pose.orientation.w])
        self.target_ori = self.tool_flip * tgt

        # Decide whether to pre-flip the base
        if self.target_pose[0] < 0:
            # Rotate base joint (joint1) by 180 deg first
            self.flip_required = True
            self.q1_target = wrap_to_pi(self.q[0] + np.pi)
            self.get_logger().info("Target x<0: scheduling base 180° pre-flip.")
        else:
            self.flip_required = False

        rv = self.target_ori.as_rotvec()
        self.get_logger().info(f"Desired position: {self.target_pose}")
        self.get_logger().info(f"Desired orientation (rad): {np.round(rv, 4)}")

    def control_loop(self):
        if self.target_pose is None or not self.q_initialized:
            return

        # Phase 1: base pre-flip if needed
        if self.flip_required:
            err = wrap_to_pi(self.q1_target - self.q[0])
            if abs(err) > self.base_flip_tol:
                qdot = np.zeros(7)
                u = np.clip(self.base_flip_gain * err,
                            -self.base_flip_max_speed, self.base_flip_max_speed)
                qdot[0] = u
                self.control = 'Base pre-flip'
                self._integrate_and_publish(qdot)
                return
            else:
                self.flip_required = False  # pre-flip done

        # Phase 2: normal resolved-rates with null-space KMI+JLA
        x_cur, R_cur, _ = forward_kinematics(self.q)
        e = self.target_pose - x_cur
        R_current = R.from_matrix(R_cur)
        R_err = self.target_ori * R_current.inv()
        omega = R_err.as_rotvec()

        dist_x = np.linalg.norm(e)
        dist_R = np.linalg.norm(omega)

        if dist_x < 0.03 and dist_R < 0.03:
            qdot = np.zeros(7)
            if not self.goal_reached:
                rotvec_final = R_current.as_rotvec()
                self.get_logger().info(f"Final position: {x_cur}")
                self.get_logger().info(f"Final orientation (rad): {rotvec_final}")
                self.get_logger().info(f"Final joint state: {np.round(self.q,3)}")
                self.get_logger().info(f"Controller used: {self.control}")
                if self.start_time is not None:
                    end_time = self.get_clock().now()
                    elapsed = (end_time - self.start_time).nanoseconds / 1e9
                    self.get_logger().info(f"Time to reach target: {elapsed:.3f} seconds")
                    self.start_time = None
                self.goal_reached = True
            self.control = 'Hold'
            self._integrate_and_publish(qdot)
            return

        v_des = np.hstack((self.kp_x * e, self.kp_r * omega))
        v_norm = np.linalg.norm(v_des)
        if v_norm > self.max_vel:
            v_des *= self.max_vel / v_norm

        # Primary task
        J, _ = compute_jacobian(self.q)
        J_pinv = np.linalg.pinv(J)
        qdot_task = J_pinv @ v_des

        # Null space projection
        null_proj = np.zeros(7)
        hybrid = True
        # null-space cost weights
        self.a, self.b = 1, 1   
        if hybrid:
            grad_kmi = cost_gradient_KMI(self.q)
            grad_jla = cost_gradient_JLA(self.q)
            grad_total = self.a * grad_kmi + self.b * grad_jla
            null_proj = (np.eye(7) - J_pinv @ J) @ grad_total
            if self.a == 0 and self.b != 0:
                self.control = 'JLA'
            elif self.a != 0 and self.b == 0:
                self.control = 'KMI'
            else:
                self.control = 'Hybrid'
        else:
            self.control = 'No cost function'

        qdot = qdot_task + null_proj
        self._integrate_and_publish(qdot)

    def _integrate_and_publish(self, qdot):
        # integrate
        self.q += qdot / self.rate
        # publish velocity
        self.vel_pub.publish(Float64MultiArray(data=qdot.tolist()))
        # publish joint state
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = [f'joint{i+1}' for i in range(7)]
        js.position = self.q.tolist()
        js.velocity = qdot.tolist()
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = XArmResolvedRate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()