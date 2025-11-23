from roboticstoolbox import ERobot
from spatialmath import SE3
import numpy as np

def validate_joint_configuration(position, o_vector, a_vector, q_sol):
    if q_sol is None:
        print("No joint solution to validate.")
        return False

    robot = ERobot.URDF("/home/danielrsouza/robotics-toolbox-python/xarm7.urdf")

    deg_limits = [
        (-360, 360), (-118, 120), (-360, 360), (-11, 225),
        (-360, 360), (-97, 180), (-360, 360)
    ]
    rad_limits = np.deg2rad(deg_limits)

    joints = [link for link in robot.links if link.isjoint]
    for j, (min_rad, max_rad) in zip(joints, rad_limits):
        j.qlim = [min_rad, max_rad]

    desired_pose = SE3(*position) * SE3.OA(o_vector, a_vector)

    within_limits = all(j.qlim[0] <= q <= j.qlim[1] for q, j in zip(q_sol, joints))
    T_actual = robot.fkine(q_sol)

    pos_error = np.linalg.norm(T_actual.t - desired_pose.t)
    rot_error = np.linalg.norm(T_actual.R - desired_pose.R, ord='fro')

    pos_tol = 1e-3
    rot_tol = 1e-3

    position_ok = pos_error < pos_tol
    orientation_ok = rot_error < rot_tol

    print("\n--- Validation ---")
    print("Reachable:", within_limits and position_ok and orientation_ok)
    print("Joint angles (rad):", q_sol)
    print("Position error (m):", pos_error)
    print("Orientation error (Frobenius norm):", rot_error)

    return within_limits and position_ok and orientation_ok
