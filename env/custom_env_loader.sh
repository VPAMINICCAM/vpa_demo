#!/bin/bash
# Source the ROS setup
source /opt/ros/noetic/setup.bash

# Source the catkin workspace
source /home/vpaadmin/catkin_ws/devel/setup.bash

# Export custom environment variables
export ROBOT_NAME=$(hostname)  # Uses the hostname as the robot name, or replace $(hostname) with specific names if needed

# You can add other necessary environment variables or commands here

exec "$@"
