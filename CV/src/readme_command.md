ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true

./setupRealur5e.sh (With gripper run: ros2 launch ur_with_gripper_description real_robot_bringup.launch.py )

ros2 run my_robot_cv object_recognizer

ros2 run robot_arm_mover robot_arm_mover

