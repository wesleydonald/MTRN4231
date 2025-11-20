#!/bin/bash

gnome-terminal -t 'DriverServer' -e "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false"

sleep 10

gnome-terminal -t 'MoveitServer' -e "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true"

