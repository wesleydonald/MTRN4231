#!/bin/bash

gnome-terminal -t 'MoveItServer' -e "ros2 launch arm moveit_fake.launch.py"
gnome-terminal -t 'DriverServer' -e "ros2 launch arm ur_control_fake.launch.py"
gnome-terminal -t 'Ros2Nodes' -e "ros2 launch brain fake.launch.py"
