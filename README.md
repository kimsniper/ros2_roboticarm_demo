# Robotic Arm (3 DOF) Demo for ROS2

## Overview

This reposotory contains a robotic arm demo with actual hardware integration PCA9685 to actuate joint servo motors. It contains the driver and node to driver LEDs/Servo motor. The PCA9685 in interfaced to a Raspberry Pi 4 board running Ubuntu with ROS2 installed.

## Hardware

This package has been tested with the following hardwares: \

Raspberry Pi 4B, 4GB RAM. \
PCA9685 sensor breakout board used: https://www.nxp.com/products/power-management/lighting-driver-and-controller-ics/led-controllers/16-channel-12-bit-pwm-fm-plus-ic-bus-led-controller:PCA9685 \
Towerpro SG90 Servo motor: https://www.towerpro.com.tw/product/sg90-7/

### Preferred Environment Setup (Host PC)

To run this example without issues, the following environment setup are preferred.

|                  |                          |
|------------------|--------------------------|
| Operating System | Ubuntu 22.04             |
| ROS2 Version     | ROS2 Humble              |

### Preferred Environment Setup (Raspberry Pi 4)

To run this example without issues, the following environment setup are preferred.

|                  |                                                          |
|------------------|----------------------------------------------------------|
| Operating System | https://github.com/ros-realtime/ros-realtime-rpi4-image  |

Note: Since the RPi4 will run an Ubuntu Server, the PC will communicate with Raspberry Pi 4 via SSH. 

### Workspaces

**This repository contains two ROS2 workspaces, the workspace that will run on RPI4 (roboticarm_pi_ros2_ws) and on PC/Host (roboticarm_ros2_ws). Just clone this repository on both PC and Raspberry Pi 4 as shown below:

```bash
git clone --recursive https://github.com/kimsniper/ros2_roboticarm_demo.git
```

### Instructions for PC ROS2 workspace

1. Navigate to < Repo path on your PC >/ros2_roboticarm_demo/roboticarm_ros2_ws
2. Build the packages
```bash
colcon build
```
3. Source the setup.bash file
```bash
source install/setup.bash
```
4. Launch Gazebo
```bash
ros2 launch roboticarm_bringup gazebo.launch.py
```
5. In another terminal, launch the Joint State Publisher GUI
```bash
ros2 launch roboticarm_bringup slider_controller.launch.py
```
The actual robot and the Gazebo-simulated one can now be actuated by the Joint State Publisher GUI

Or if you want to test the MoveIt2 demo, follow the following instructions.

In another terminal, launch the controller
```bash
ros2 launch roboticarm_bringup controller.launch.py
```
And in another terminal, launch MoveIt2 (via RViz2)
```bash
ros2 launch roboticarm_bringup moveit.launch.py
```
The actual robot and the Gazebo-simulated one can now be actuated using MoveIt2 through Rviz2 GUI 

Important: In Rviz2 GUI->Motion Planning Window->Context tab, set CHOMP to OMPL

Note: The Joint State Publisher control and MoveIt control should never be used/run at the same time.

### Instructions for Raspberry Pi 4 ROS2 workspace

1. Navigate to < Repo path on your PC >/ros2_roboticarm_demo/roboticarm_pi_ros2_ws
2. Build the packages
```bash
colcon build
```
3. Source the setup.bash file
```bash
source install/setup.bash
```
4. Run the PCA9685 node
```bash
ros2 launch ros2_pca9685 ros2_pca9685.launch.py
```
5. In another terminal, run the joint processor node
```bash
ros2 run ros2_jointprocessor ros2_jointprocessor
```
The nodes should be running by now.

### Working Demo Videos

Inverse Kinematics Demo with MoveIt2: https://www.linkedin.com/posts/activity-7198200785514500096-aaFl?utm_source=share&utm_medium=member_desktop

Interactive Robotic Arm Control: https://www.linkedin.com/posts/activity-7198545377602461696-PqYb?utm_source=share&utm_medium=member_desktop
