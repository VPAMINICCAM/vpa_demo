#!/bin/bash
# Source the ROS setup
source /opt/ros/noetic/setup.bash

# Source the catkin workspace
source /home/vpaadmin/catkin_ws/devel/setup.bash

# Source needed
source /home/vpaadmin/.bashrc
# You can add other necessary environment variables or commands here

roslaunch vpa_demo fast_start_smart_intersection.launch

