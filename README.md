# xArm7 Motion Control (ROS2)

This package implements joint-level and end-effector velocity control for the xArm7 robotic manipulator.  
It was developed as part of research work in Stevens Institute of Technology’s ARMLab and is designed to run alongside the official xArm ROS packages.

---

## Features

### Joint Velocity Control
- Commands joint velocities directly to the robot through ROS2 publishers.
- Includes basic safety checks for joint limits and self-collision.

### End-Effector Pose Tracking
- Computes end-effector velocity commands that follow a desired Cartesian pose.
- Uses Jacobian-based differential kinematics.

### ✔ Joint Configuration Validation
- Ensures generated joint angles are feasible, within limits, and non-colliding.

---

## Dependencies

- You must install the official xArm ROS packages: https://github.com/xArm-Developer/xarm_ros
- Created with: Ubuntu 22.04, ROS2 Humble
