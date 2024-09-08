# ScanMatcher Node - Instructions

## Overview
The ScanMatcher node is a ROS2 node that subscribes to laser scan data (/scan) and odometry data (/odom). It processes the laser scan to match features with a preloaded map and calculates the robot's yaw change. It publishes the yaw to the /sml_yaw topic and commands the robot to rotate and move forward based on the calculated yaw difference.

The node also generates images for debugging purposes, including edge-detected sections of the map and laser scan images.

## Prerequisites

ROS2 installed and sourced (e.g., Humble or later).
Required libraries installed (e.g., OpenCV, YAML-CPP).
To install OpenCV and YAML-CPP, run:

```bash
sudo apt install libopencv-dev
sudo apt install libyaml-cpp-dev
```
## How to Build the Package

Navigate to the root of your ROS2 workspace:
```bash
cd ~/ros2_ws
```
Add the ScanMatcher code to a package (e.g., scan_matcher_package).

## Build the package:
```bash
colcon build --packages-select scan_matcher_package
```
## Source the workspace:
source install/setup.bash

## How to Run the Node

Ensure you have a valid map file in YAML format (e.g., map.yaml), which specifies the resolution and origin of the map and references an associated image file (e.g., map.pgm).

Run the node, providing the path to the map YAML file:
```bash
ros2 run scan_matcher_package scan_matcher /path/to/map.yaml
```
The node will subscribe to /scan and /odom, perform scan matching, and publish the calculated yaw angle to the /sml_yaw topic.

##Command-line Example
```bash
ros2 run scan_matcher_package scan_matcher 
~/ros2_ws/maps/my_map.yaml
```
## Topics Used

/scan: Laser scan data (input).
/odom: Odometry data (input).
/cmd_vel: Robot velocity commands (output).
/sml_yaw: Yaw angle in degrees (output).
Files Generated

first_image_edges.png: Image showing edges of the map section around the robot.
second_image.png: Image representing the processed laser scan data.
These files are saved in the directory where the node is executed.

## Notes

The node will calculate yaw changes and control the robot’s rotation and movement.
Ensure the map YAML file is correctly formatted and accessible.
Check the output directory for debugging images (first_image_edges.png and second_image.png).
Dependencies

ROS2 (rclcpp)
OpenCV for image processing
YAML-CPP for loading map files
Standard ROS2 message types like LaserScan, Twist, Odometry, and Float64
