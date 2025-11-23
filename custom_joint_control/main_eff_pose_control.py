import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from custom_joint_control.generate_joint_angles import generate_joint_angles
from custom_joint_control.validate_joint_configuration import validate_joint_configuration
from scipy.spatial.transform import Rotation as R


class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/target_pose', 10)
        self.pose_msg = None
        self.pose_sent = False
        self.timer = self.create_timer(1.0, self.timer_callback)  # check every second

    def publish_pose(self, pos, rot_matrix):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = pos

        quat = R.from_matrix(rot_matrix).as_quat()
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.pose_msg = msg
        self.pose_sent = False
        self.get_logger().info("Pose message stored for publishing...")

    def timer_callback(self):
        if self.pose_msg and not self.pose_sent:
            self.pose_msg.header.stamp = self.get_clock().now().to_msg()  # refresh timestamp
            self.publisher.publish(self.pose_msg)
            self.pose_sent = True
            self.get_logger().info("Published target pose to /target_pose")

def get_user_pose():
    pos_str = input("Enter desired position (x y z): ")
    euler_str = input("Enter orientation (Euler XYZ in degrees): ")

    position = list(map(float, pos_str.strip().split()))
    euler_deg = list(map(float, euler_str.strip().split()))

    # Convert Euler angles to rotation matrix
    rotation = R.from_euler('xyz', euler_deg, degrees=True)
    rotation_matrix = rotation.as_matrix()

    return position, rotation_matrix


def main():
    rclpy.init()
    node = PosePublisher()

    # Define Target Pose
    #position, rotation_matrix = get_user_pose()
    position = [0.35, 0.25, 0.1]
    euler_deg = [0, 0, 0]  

    rotation_matrix = R.from_euler('xyz', euler_deg, degrees=True).as_matrix()
    
    o_vector = rotation_matrix[:, 0] 
    a_vector = rotation_matrix[:, 2]  
    q_sol = generate_joint_angles(position, o_vector, a_vector)

    if validate_joint_configuration(position, o_vector, a_vector, q_sol):
        print("\nConfiguration is valid. Publishing...\n")
        node.publish_pose(position, rotation_matrix)

        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            print("Shutting down pose publisher...")
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        print("\nConfiguration is invalid. Aborting.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()