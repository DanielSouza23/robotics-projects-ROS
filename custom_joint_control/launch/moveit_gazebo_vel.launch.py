import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Namespace for hardware (matches xarm_gazebo defaults)
    hw_ns = LaunchConfiguration('hw_ns', default='xarm')

    # Include the stock Gazebo + MoveIt launch
    robot_moveit_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('xarm_moveit_config'),
                'launch',
                '_robot_moveit_gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'dof': '7',
            'robot_type': 'xarm',
            'hw_ns': hw_ns,
            'no_gui_ctrl': 'false',
        }.items(),
    )
    
    return LaunchDescription([
        robot_moveit_gazebo,
    ])