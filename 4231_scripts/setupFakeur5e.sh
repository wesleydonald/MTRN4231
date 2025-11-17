#!/bin/bash

SESSION_NAME="fake_ur5e"

tmux has-session -t "$SESSION_NAME"

if [ $? != 0 ]; then
  tmux new-session -s "$SESSION_NAME" -d

  # Rename the first window (default window 0)
  tmux rename-window -t "$SESSION_NAME:1" "DriverServer"
  tmux send-keys -t "$SESSION_NAME:1" "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true launch_rviz:=false" C-m

  # Create a new window named "Editor" and run vim
  tmux new-window -t "$SESSION_NAME:2" -n "MoveitServer"
  tmux send-keys -t "$SESSION_NAME:2" "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true" C-m

  # Select the first window ("Main")
  tmux select-window -t "$SESSION_NAME:0"
fi

# Attach to the tmux session
tmux attach-session -t "$SESSION_NAME"
