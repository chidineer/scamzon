sml_graph Node - Instructions

Overview
The sml_graph package contains a ROS2 node that subscribes to the /sml_yaw topic, records yaw data over a period of 60 seconds, and saves the recorded data in a CSV file. Additionally, the node generates a graph plotting yaw versus time and saves it as a PNG file.

Generated Files:

yaw_data.csv - Contains time-stamped yaw data in degrees.
yaw_plot.png - A graphical plot of yaw versus time.
Prerequisites

Ensure ROS2 is installed and sourced.

Install the required Python library matplotlib using:

pip3 install matplotlib

How to Build the Package

Navigate to the root of your ROS2 workspace (e.g., ~/ros2_ws/).
Run the following command to build the sml_graph package:
colcon build --packages-select sml_graph

Source the workspace:
source install/setup.bash

How to Run the Node

Run the yaw_listener node to start recording yaw data:
ros2 run sml_graph yaw_listener

The node will record yaw data from the /sml_yaw topic for 60 seconds, then generate the CSV and graph files.
Generated Files Location
Both files will be saved in the output/ directory inside the sml_graph package.

CSV file: ~/ros2_ws/src/sml_graph/output/yaw_data.csv
Graph file: ~/ros2_ws/src/sml_graph/output/yaw_plot.png
Viewing the Output

CSV Data
The yaw_data.csv file contains two columns:
Time (s) - The elapsed time in seconds.
Yaw Angle (degrees) - The yaw angle received from the /sml_yaw topic.
Open the CSV file using a text editor or spreadsheet application.

Graph
The yaw_plot.png file contains a graph with yaw (in degrees) on the Y-axis and time (in seconds) on the X-axis. The Y-axis is limited to a range of -180 to 180 degrees.
Notes

The node will automatically stop after 60 seconds.
Ensure the /sml_yaw topic is being published by another ROS2 node calculating the yaw.