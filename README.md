# xArm7 Motion Control (ROS2)

This package implements joint-level and end-effector velocity control for the xArm7 robotic manipulator.  
It was developed as part of research work in Stevens Institute of Technology’s ARMLab and is designed to run alongside the official xArm ROS packages.

---

## Features

### End-Effector Pose Tracking
- Computes end-effector velocity commands that follow a desired Cartesian pose - establish on main code file.
- Uses Jacobian-based differential kinematics.

### Joint Velocity Control
- Commands joint velocities directly to the robot through ROS2 publishers.
- Includes basic safety checks for joint limits and self-collision.

### Joint Configuration Validation
- Ensures generated joint angles are feasible, within limits, and non-colliding.

---

## Dependencies

- You must install the official xArm ROS packages: https://github.com/xArm-Developer/xarm_ros  
- Created with: Ubuntu 22.04, ROS2 Humble

---

## How to Run

After building and sourcing the workspace, run two terminals.

### Terminal 1 - Launch xArm RViz simulation (from xarm_ros package)
ros2 launch xarm_description xarm7_rviz_display.launch.py

### Terminal 2 - Launch this package's control node
ros2 launch custom_joint_control eff_control.launch.py


