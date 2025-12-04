#!/bin/bash

gnome-terminal -t 'MoveItServer' -- ros2 launch arm moveit_fake.launch.py
gnome-terminal -t 'DriverServer' -- ros2 launch arm ur_control_fake.launch.py

read -p "Press enter to launch ros nodes."

gnome-terminal -t 'Ros2Nodes' -- ros2 launch brain fake.launch.py
