from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    script_dir = '/home/danielrsouza/dev_ws/src/custom_joint_control/custom_joint_control'

    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', os.path.join(script_dir, 'main_eff_pose_control.py')],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['python3', os.path.join(script_dir, 'force_velocity_adv.py')],
            output='screen'
        ),
    ])
