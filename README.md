<!-- omit from toc -->
# MTRN4231 - TICTACTOE BOT

- [Introduction](#introduction)
  - [Bugs/TODOs](#bugstodos)
  - [Video Demonstration](#video-demonstration)
- [ROS Packages](#ros-packages)
  - [Brain](#brain)
  - [Arm](#arm)
  - [End Effector](#end-effector)
  - [Visualisation](#visualisation)
  - [Simulation](#simulation)
  - [Interfaces](#interfaces)
  - [Computer Vision](#computer-vision)
  - [Depth Camera](#depth-camera)
- [Feature Overview](#feature-overview)
  - [Piece/Board Detection](#pieceboard-detection)
    - [To test on a video](#to-test-on-a-video)
    - [To test on an image](#to-test-on-an-image)
  - [To run tests with end-effector connected](#to-run-tests-with-end-effector-connected)
  - [To run tests without end-effector connected](#to-run-tests-without-end-effector-connected)
  - [Custom End-effector](#custom-end-effector)

## Introduction

The project utilizes UR5e from Universal Robots to play a game of tictactoe.

**Project Duration**: 8 weeks

**Authors**:
* Ryan
* Shanrui
* Wesley

### Bugs/TODOs
- TODO: (Arm Node)The arm planning is still way too slow ~1 minute to make a move and get back home, need to speed this up.
- TODO: (Arm Node) Stop the z axis rotating during movements.
- TODO: (Arm Node) I wonder if there is a way to preplan paths like query moveit in advance to speed up the process.
- TODO: (Arm Node) Still shows the attached object geometry (found this really hard to remove).
- TODO: (Brain Node) The robot should lift the piece higher off the ground before moving to place it.
- TODO: (Brain Node) Increase the cell size to factor in borders ~110mm instead of ~100mm.
- TODO: (Brain Node) Make sure the gripper node can handle error cases such as when the robot fails to plan a valid path.
- TODO: (End Effector Node) Ensure there is protections against closing or opening the gripper twice in a row.
- TODO: (End Effector Node) Make the marker dynamically update to show the robot picking up a piece in rviz.
- TODO: (End Effector Node) Currently the GripperOpen.stl file is in the wrong orientation (not sure why) so need to fix that.
- TODO: (Simulation) Change the comment in the simulator node (currently its just the brain nodes comment).
- TODO: (Simulation) Make sure the simulation node doesn't run when we are in the lab.
- TODO: (CV Node) The computer vision node should detect unknown objects, publish them to a topic and the arm node should subscribe to dyamically create obstacles.
- TODO: (General) Write a script to simplify the startup process (instead of running each package in a new terminal) need one for in and out of the lab.
- TODO: (General) Change "chessboard" to "board" everywhere.
- TODO: (General) Account for the fact that the board can start at any rotation (assume it doesnt move during the game though).
- TODO: (README) Write up the computer vision and depth camera sections.
- TODO: (README) Make sure all the publisher/subscriber relationships show the message types. Maybe making custom types could decrease the number of publisher/subscriber relationships we need.

### Video Demonstration
[demo](https://drive.google.com/file/d/1wB0pO2BWPVrgC5FvZPbpUHrPBrPJTyYX/view?usp=drive_link)

## ROS Packages
### Brain

Controls the ur5 to play a game of tictactoe. The idea is we are in the PLAYER_TURN state until we see a piece has moved onto the board. Then we calculate the best move. We then enter the ROBOT_TURN state where we send a service request to the arm to move to a certain x, y ocation. When we recieve the completed response from the arm node we send a service request to the gripper node to close the gripper. Then a similar thing for lifting the piece and placing it on the board. Lastly we return to home_pose and wait for the next move. A TicTacToe class is implemented for the gameplay logic. NOTE THIS NODE IS NOT FULLY TESTED.

Subscribes to:
  - /detected/board
  - /detected/pieces/white
  - /detected/pieces/black

A client to:
  - gripper node
  - arm node

Assumptions
  - Human plays first with X's
  - The /detected topics publish in the robot base frame
  - Note: a bool using_gripper is implemented in this file so testing can be done without the arduino

### Arm 

Launches moveit and interacts with it to plan paths for the robot. An orientation constraint has been implemented as the end effector should always be straight down for our implementation. Sets up collision objects and uses an incremental planning time to decrease the overall planning time for simple moves.

A service to:
  - brain node
  
### End Effector  

Sends a string command to the arduino through a serial port. Recieves a command from the brain node and sends it to the arduino. Commands are specifically "open\n" and "close\n".

A service to:
  - brain node

Assumptions
  - Possible commands from brain node are "open" and "close".

### Visualisation

Publishes markers to rviz for system visualisation. Currently this node just publishes the markers and end effector to their positions. Will later add a more dynamic visualisation that can show the piece being picked up with the end effector (exciting).

Subscribes to:
  - /detected/board
  - /detected/pieces/white
  - /detected/pieces/black

Publishes to:
  - /tictactoe/white_markers (marker array)
  - /tictactoe/black_markers (marker array)
  - /tictactoe/board_marker (single marker)
  - /tictactoe/gripper_marker (single marker)

### Simulation
This package publishes to the board and pieces topic, so we can use this when we are out of the lab. A timer can be used to "move a piece" which can then test that the visualisation and brain node is working I think this will be useful for future developments as well.

Publishes to:
  - /detected/board
  - /detected/pieces/white
  - /detected/pieces/black

### Interfaces

Used for custom messages and services.
  - CloseGripper.srv
  - MoveArm.srv

### Computer Vision

Object detection and pose approximation using the depth camera. Subscribes to the depth camera information then publishes piece and baord markers to the calculated pose of the pieces and board.

#### Subscribers -  
  - `/camera/camera/color/image_raw`
  - `/camera/camera/aligned_depth_to_color/image_raw`
  - `/camera/camera/aligned_depth_to_color/camera_info`

#### Publishers -
**Detection**
  - `/detected/board` (point)
  - `/detected/pieces/white` (pose array)
  - `/detected/pieces/black` (pose marker)

**Debugging**
  - `/debug/white_piece_detection` (image)
  - `/debug/black_piece_detection` (image)
  - `/debug/board_detection` (image)
  - `/debug/detection_image` (image)

To run the object detection and realsense camera setup (best for real robot) use:

``` bash
ros2 launch cv cv_real.launch.py
```

To run the object detection on its own and provide video manaully (best for simulation) use:
``` bash
ros2 run cv object_detection.py
```

### Depth Camera

Realsense camera launch script: 

```
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true
```

## Feature Overview

### Piece/Board Detection
TODO: This section to show how we use CV

#### To test on a video
TODO: Example command
#### To test on an image
TODO: Example command

### To run tests with end-effector connected
TODO: This stuff is not implemented just copied from the example
**Step 1**: In one terminal, run `ros2 launch brain system_launch.py`
**Step 2**: In another terminal, run `ros2 run testing brain_vision_test`, or other testing files

### To run tests without end-effector connected
TODO: This stuff is not implemented just copied from the example
**Step 1**: In one terminal, run `ros2 launch brain without_endeffector_launch.py`
**Step 2**: In another terminal, run `ros2 run testing brain_vision_test`, or other testing files

### End-effector visualisation
TODO: This stuff is not implemented just copied from the example
1. `use_fake` indicates whether Real or Fake UR5e is used
2. There are launch files that launches only the end-effector or the end-effector with UR5e with no driver support
   1. `ros2 launch end_effector_description end_effector_only.launch.py`
   2. `ros2 launch end_effector_description end_effector_withModel.launch.py`

### Custom End-effector
TODO: Insert end effector design
