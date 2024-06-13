#!/bin/bash

# This script sets up the environment and launches a ROS node.

# Source the ROS Noetic setup to configure the environment.
source /opt/ros/noetic/setup.bash

# Source the development setup for the Catkin workspace.
source /home/vpaadmin/catkin_ws/devel/setup.bash

# Set the ROS networking environment variables.
export ROS_IP=$(hostname -I | awk '{print $1}')
export ROS_HOSTNAME=$(hostname -I | awk '{print $1}')
export ROS_MASTER_URI=http://192.168.1.4:11311

# Set a custom environment variable for the robot's name based on the hostname.
export robot_name=$(hostname)

# Additional environment variables or commands can be added below this line.

# Launch the ROS node with a specific robot name derived from the hostname.
roslaunch vpa_demo fast_start_smart_intersection.launch
