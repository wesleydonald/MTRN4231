ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true

./setupFakeur5e.sh

ros2 run my_robot_cv object_recognizer

ros2 run robot_arm_mover robot_arm_mover

