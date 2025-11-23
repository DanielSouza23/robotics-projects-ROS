from roboticstoolbox import ERobot
from spatialmath import SE3

def generate_joint_angles(position, o_vector, a_vector):

    robot = ERobot.URDF("/home/danielrsouza/robotics-toolbox-python/xarm7.urdf")

    desired_pose = SE3(*position) * SE3.OA(o_vector, a_vector)

    try:
        sol = robot.ikine_LM(desired_pose)
        return sol.q if sol.success else None
    except Exception as e:
        print("IK Error:", str(e))
        return None
