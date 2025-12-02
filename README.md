<!-- omit from toc -->
# MTRN4231 - Capstone Mechatronics Project 

[README Checklist](https://docs.google.com/document/d/1zuUf0pBpQOLhTJfOn6BVm4h70meYy5rnrdafGd6j8Xo/edit?tab=t.0)

![banner](images/banner.png)

# Table of Contents
1. [Project Overview](#project-overview)
2. [System Architecture](#system-architecture)
   - [ROS2 Architecture](#ros2-architecture)
   - [State Machine](#state-machine)
4. [Technical Components](#technical-components)
   - [Computer Vision](#computer-vision)
   - [Custom End-Effector](#custom-end-effector)
   - [System Visualisation](#system-visualisation)
   - [Closed-Loop Operation](#closed-loop-operation)
5. [Installation and Setup](#installation-and-setup)
6. [Running the System](#running-the-system)
7. [Results and Demonstration](#results-and-demonstration)
8. [Discussion and Future Work](#discussion-and-future-work)
9. [Contributors and Roles](#contributors-and-roles)
10. [Repository Structure](#repository-structure)
11. [References and Acknowledgements](#references-and-acknowledgements)

---

# Project Overview 
Intended customer / end-user.  
Summary of robot functionality.  
Short [demo](https://drive.google.com/drive/folders/1qhxOtTVrk9HNywdLUwSORVOnxeD5zi6X)

---

# System Architecture

## ROS2 Architecture
- Diagram of ROS2 nodes, topics, services, and actions. 
- Package-level architecture diagram.
- Description of each node’s function.  
- List and explanation of any custom message types or interfaces.

 ## State Machine

The system is controlled by a two-tier state machine. At the top level, the game progresses through three high-level states, `PLAYER_TURN`, `ROBOT_TURN` and `FINISHED`. These high-level states describe the game flow, while a secondary set of action states manages the robot’s physical motion.
  
<p align="center">
  <img src="images/Robot_States.png" width="800">
</p>

During `ROBOT_TURN`, the system enters a sequence of low-level action states that move the robot through the pick-and-place routine. Each action triggers a ROS 2 service call to either the arm or gripper node. Once the service responds successfully, the state machine transitions to the next action until the full sequence completes. When all actions succeed, the system returns the arm to its home position and switches back to `IDLE`.

<p align="center">
  <img src="images/Turn_States.png" width="800">
</p>

If any arm or gripper service reports a failure, the current action sequence is immediately aborted. The system attempts to move the robot to its home position to prevent undefined behaviour. The high-level state machine transitions to `FINISHED` and `IDLE`.

---

# Technical Components

## Computer Vision
Description of the vision pipeline and its contribution to the task.

## Custom End-Effector
The follow engineering drawings are to AS1100 and include all the dimensions required to reproduce the end effector.

<p align="center">
  <img src="images/Chassis_Drawing.png" width="600">
</p>

<p align="center">
  <img src="images/Gripper_Drawing.png" width="600">
</p>

<p align="center">
  <img src="images/Holder_Drawing.png" width="600">
</p>

<p align="center">
  <img src="images/Assembly_Drawing.png" width="600">
</p>

All manufactured parts can be printed with PLA. The DSS-P05 servo is secured to the chassis using four 2.5 mm bolts. One gripper arm is bonded to a standard servo horn using two-part epoxy. The second gripper arm is mounted directly to the chassis using a 2.5 mm bolt. The interchangeable holders can be attached to the gripper simply via a friction fit. A full render and image of the assembled end effector is shown below.

<p align="center">
  <img src="images/render.png" height="500" style="vertical-align: middle; margin-right: 20px;">
  <img src="images/Gripper.png" height="500" style="vertical-align: middle;">
</p>

The end effector is actuated using a DSS-P05 servo, controlled through a ROS 2 node (`gripper`) that communicates with an Arduino over a serial connection. The node exposes a custom service (`/gripper_service`, type `CloseGripper`) which accepts either `"open"` or `"close"` as valid commands.

When the node starts, it attempts to open the serial port (`/dev/ttyUSB0`). If successful, commands recieved through the service are forwarded directly to the Arduino, which actuates the servo accordingly. If the Arduino is not connected, the node logs a warning but remains active so that the rest of the system can continue operating.

Mechanically, the chassis mounts to the UR5 wrist using a Housing Block and an End Effector Mount. The end effector’s mass is included in the URDF to ensure accurate planning and simulation. The URDF also defines collision geometry and a mesh to visualise the end effector in rviz.

## System Visualisation
The system is visualised in RViz, where both the UR5e robot and the custom gripper URDF are loaded through a combined launch sequence that starts the UR driver, MoveIt, and RViz with a custom configuration. RViz displays the full robot model alongside several marker topics published by the visualisation node, which provides a live representation of the detected board as well as the black and white game pieces. These markers are generated from perception inputs (/detected/board, /detected/pieces/white, /detected/pieces/black) and rendered as mesh resources, allowing the operator to observe the system’s understanding of the game state. This visualisation demonstrates the TF alignment of the board as well.

<p align="center">
  <img src="images/Visualisation.png" width="600">
</p>

## Closed-Loop Operation
Description of feedback method and real-time behavioural adaptation.

---

# Installation and Setup
Step-by-step installation instructions, dependencies, workspace setup.  
Hardware setup details (UR5e, camera, Teensy, etc.).  
Environment variables, config files, calibration procedures.

---

# Running the System
Instructions for launching and running the complete system.  
Example commands.  
Expected behaviour and example outputs.  
Optional troubleshooting notes.  
System must launch via a single command.

---

# Results and Demonstration
Performance relative to design goals.  
Quantitative results (accuracy, repeatability).  
Photos/figures/videos of system operation.  
Notes on robustness, adaptability, innovation.

<p align="center">
  <img src="images/pickup.png" width="400">
</p>

---

# Discussion and Future Work
Engineering challenges and how they were addressed.  
Opportunities for improvement (Version 2.0).  
What makes the approach novel or effective.

---

# Contributors and Roles
List team members and their responsibilities.

Ryan - Computer Vision, End effector design, Moveit constraints

Sherry - End effector package, UDRF + Launch files

Wesley - End effector CAD, Moveit path planning

---

# Repository Structure
The repository structure is as follows:

```
images/
src/
   arm/
   brain/
   cv/
   end_effector/
   interfaces/
   simulation/
   visualisation/
   webui/
```

The `src/` folder includes the core ROS 2 packages: `arm/` handles motion planning and robot control, `brain/` manages the game logic and state machine, `cv/` performs board detection, `end_effector/` controls the gripper, `interfaces/` defines custom message and service types, `simulation/` simulates computer vision, `visualisation/` publishes markers to rviz, and `webui hosts/` the user interface. The `images/` directory contains all images used in the report.

---

# References and Acknowledgements
This project uses several external tools and libraries, including OpenCV for computer vision, pyserial for Arduino communication, RViz2 for visualisation, and Gazebo for simulation. We also acknowledge the UR driver and MoveIt frameworks that support the robot control pipeline. We would like to thank the demonstrators David, Alex, and Saba for their consistent guidance and support throughout the project.
