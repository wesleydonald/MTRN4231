# ROS2 Chessboard and Piece Detector for Robot Arm

This ROS2 package contains a single node, object_recognizer, designed to work with a robot arm and an Intel RealSense depth camera. The node identifies a chessboard and game pieces (white round, black cross) within a specified region of interest, calculates their 3D positions in the robot's world frame, and publishes this information for use in pick-and-place tasks.

# Features

Chessboard Detection: Identifies the largest rectangular contour in the ROI as the chessboard.

Piece Detection:

Detects white, circular pieces using Hough Circle Transform.

Detects black, cross-shaped pieces using color and solidity filtering.

Region of Interest (ROI): All processing is confined to a user-defined ROI for improved performance and accuracy.

2D to 3D Conversion: Uses depth data from a RealSense camera to convert 2D pixel coordinates into 3D points.

Coordinate Transformation: Transforms 3D points from the camera's frame to the robot's base_link frame.

Self-Contained Calibration: Includes a hard-coded static transform for hand-eye calibration, removing the need for an external static_transform_publisher.

Data Publishing:

Publishes the board's 3D center point.

Publishes an array of all detected pieces' 3D poses.

Publishes TF frames for Rviz visualization.

Publishes a debug image showing live detections.

# Prerequisites

ROS2 Humble Hawksbill

An Intel RealSense Depth Camera

realsense-ros package installed and configured.

cv_bridge, tf2_ros, tf2_geometry_msgs (typically included in ros-humble-desktop-full).

OpenCV for Python: pip install opencv-python

NumPy: pip install numpy

# Setup and Installation

Create a ROS2 Workspace:

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws


Create the Package:

cd src
ros2 pkg create --build-type ament_python my_robot_cv


Add the Node:

Place the robot_arm_cv.py script inside the my_robot_cv/my_robot_cv/ directory.

Configure setup.py:

Edit src/my_robot_cv/setup.py to add the entry point for the node.

Inside the 'console_scripts' list:
'object_recognizer = my_robot_cv.robot_arm_cv:main',


Build the Package:

cd ~/ros2_ws
colcon build --packages-select my_robot_cv


# Configuration

Before running the node, you must configure the following parameters inside robot_arm_cv.py:

Hand-Eye Calibration:

In the publish_static_transform function, update the translation and rotation values to match your robot's specific hand-eye calibration data.

 In publish_static_transform()
t.transform.translation.x = 1.27677 # <-- YOUR VALUE
t.transform.translation.y = 0.0175114 # <-- YOUR VALUE
 ... and so on for all 7 values.


Region of Interest (ROI):

In the __init__ function, adjust the self.roi rectangle [x, y, width, height] to tightly frame the chessboard in your camera's view.

In __init__()
self.roi = [100, 50, 600, 400] # [x_start, y_start, width, height]


# Usage

Source the Workspace:

In a new terminal, source your workspace's setup file.

source ~/ros2_ws/install/setup.bash


Launch the Camera:

In the same terminal, start your RealSense camera node.

ros2 launch realsense2_camera rs_launch.py


Run the Object Recognizer Node:

In a second terminal (after sourcing the workspace), run the node:

ros2 run my_robot_cv object_recognizer

