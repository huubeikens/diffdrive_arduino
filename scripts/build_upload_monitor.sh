#!/bin/bash

source /opt/ros/foxy/setup.bash
colcon build --symlink-install
read -p "Press ANY key to continue..." 
source install/setup.bash
ros2 launch my_bot launch_robot.launch.py

