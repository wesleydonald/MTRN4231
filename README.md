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

<div align="justify">
   
---

# Project Overview 
This system addresses the problem of poor engagement in current stroke rehabilitation. Traditional therapy methods are often highly repetitive, isolating, and provide no data tracking for the patient.
Our intended "customer" is a post-stroke patient with hand weakness, personified as "Nick," a 68-year-old former engineer who loves robotics and strategy games.
Our solution combines a UR5e robotic arm with a physical game of Tic-Tac-Toe, turning therapy into an activity that feels like play.

The robot's primary functionality is to act as an intelligent, autonomous opponent in a physical game of Tic-Tac-Toe. The entire process is managed by a ROS2-based architecture.
The robot's behavior follows this sequence:
1\.  Start-up:The game is initiated via terminal, and the human player moves first.
2\.  Perception:A depth camera views the scene and detects the locations of the game board and all pieces.
3\.  Decision:Once the human's move is detected, central "brain" node processes the board state and decides on the optimal move to make.
4\.  Action:The brain sends commands to the arm and gripper. The robot uses a custom-designed, 3D-printed gripper mounted to the UR5e arm.
5\. Execution:The arm moves to the decided piece, grips it, and places it in the desired location on the board.
6\.  Loop: The arm then returns to a home position and waits for the human player to take their turn.
 
[Functionality demo](https://drive.google.com/file/d/1CPIxWg0ur2wqS3amkw4ajLEvb_dR5X6y/view?usp=drive_link)
[Visualisation demo](https://drive.google.com/file/d/1lPf4V6NlLgwOl6tRVGjNmHHzRr6m4RVj/view?usp=drive_link)

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
The computer vision uses an Intel RealSense camera to detect the board and classify the white and black playing pieces. The RGB image is processed using colour thresholding while the aligned depth camera is used to convert pixel coordinates into 3D positions in the robot’s `base_link` frame.

A trapezoidal region-of-interest (ROI) restricts detection to the table workspace. The board is identified by HSV colour thresholding, Canny edge extraction and Hough line detection, which gives both the board’s centre and orientation. The orientation is found using a *line of best fit* approach. A rolling average filter stabilises the board pose before broadcasting a TF frame for the board and its 3×3 grid (`board_index[i]`).

White and black pieces are detected separately using tuned HSV thresholds, filtering, and contour solidity/area checks to reject noise. For each valid contour, image moments provide the pixel centroid. Each piece is published as a `PoseArray` in `base_link`, and TF frames are broadcast as well.

Finally, a temporal filtering stage selects the most stable detection set across multiple frames, improving robustness to flickering elements. This pipeline provides reliable board localisation and piece detection for planning in the `brain` package. A debugging image is published to `debug/detection_image` as follows,

<p align="center">
  <img src="images/CV.png" width="600">
</p>

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
The Brain node continuously monitors the most recent positions of all pieces on and off the board. When the game starts, the locations of all elements can be anywhere in the robots reachable workspace for correct operation. The only assumption that has been made is the board will not move during the game, which would result in undefined behaviour. When the player makes a move, the Brain node uses the latest piece locations to determine which piece to pick up, ensuring correct selection even if pieces have shifted during the game.

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

The system is designed to perform against a set of strict, quantitative metrics that ensure it is responsive, accurate, and safe for the end-user.
These are the design targets the system is engineered to meet:

Response Time | Board State Detection | Target Goal:< 5 seconds | Reality: 4.52 senconds

Repeatability | Optimal Game Strategy | Target: 100% consistent | Reality: 100%

Repeatability | Piece Pickup Success | Target: ≥ 99.5% | Reality: 100%

Repeatability | Piece Placement (Std Dev) | Target: ± 0.5mm | Reality: ± 6mm |

Accuracy | Piece Placement | Target: ± 2mm | Reality:  ± 6mm |

Accuracy |  Piece Pickup | Target: ± 3mm | Reality:  ± 4mm |

## Innovation: 

1\.The first innovation is the gamification of rehabilitation. By using a sophisticated robotic arm (UR5e) to play a common strategy game, the system directly combats the primary "customer" problem: poor engagement and isolation during therapy.

2\.Adaptive Board Orientation: Leverages computer vision (CV) to detect the Tic-Tac-Toe board's angle in real-time. This allows the robot to accurately identify the grid and play correctly regardless of how the board is rotated, offering significant flexibility and robustness over systems that require a fixed, pre-calibrated orientation.

## Robustness:
Hardware: A custom-designed, 3D-printed gripper with bearing gears is used for reliable, low-friction grasping. The game pieces are also custom-designed with a small tab to ensure a consistent grip.
Software: The system runs on a modular ROS2 architecture.  A central "brain" node manages game logic and perception, decoupling tasks and making the system easier to debug and maintain.

## Adaptability:
The system uses a depth camera for perception, allowing it to detect the board and piece locations dynamically. This makes it adaptable to slight changes in board position, rather than relying on fixed, hard-coded coordinates.
The modular package design  means components can be individually upgraded or even swapped. For example, the `brain` node's Tic-Tac-Toe logic could be replaced with logic for a different game (like checkers) without redesigning the entire arm or gripper packages.


<p align="center">
  <img src="images/pickup.png" width="400">
</p>

---

# Discussion and Future Work

## Major Engineering Challenges

ROS & MoveIt! Integration: Navigating the steep learning curve of the Robot Operating System (ROS) and implementing the MoveIt! motion planning framework, which was new to many members of the team.

Collaborative Workflow: Devising an effective engineering workflow to modularize development (e.g., splitting vision, game logic, and robot control) and then successfully integrating all components from different team members into a cohesive, functional system.

Hardware Design & Prototyping: Designing a custom end-effector from the ground up. The primary challenge was optimizing the design for 3D printability, ensuring it was lightweight, functional, and could be reliably produced.

## Future Enhancements

Dynamic Obstacle Avoidance: Integrate sensors to detect and avoid unexpected obstacles, such as a player's hand, entering the workspace.

Mid-Game Board Tracking: Implement continuous board detection to compensate if the board is accidentally bumped or moved during gameplay.

Angled Surface Compensation: Extend the robot's kinematic and vision model to play on surfaces that are tilted or uneven.

Illegal Move Detection: Add CV and game state logic to identify when a human player makes an invalid move and prompt them to try again.

Interactive Feedback: Implement clearer visual or auditory feedback to communicate the robot's status, such as "It's your turn," "Invalid move," or "I win!"

Player Engagement: Introduce features like selectable difficulty levels or voice/gesture-based game commands.

## Novel Approaches

Orientation-Invariant Board Detection: The system's core novelty lies in its robustness to board placement. Using CV techniques, the robot dynamically finds the game board and calculates its precise angle of rotation in real-time. This eliminates the need for a fixed, perfectly aligned camera or board, allowing a user to simply place the board at any angle and start playing.

Unbeatable Game AI: The robot isn't just a physical mover; it's a perfect player. It leverages the Minimax algorithm, a classic search algorithm from game theory, to analyze all possible board states. This ensures the robot always plays an optimal move, either winning or forcing a draw.

Modular & Decoupled Architecture: The system is built with a clear separation of concerns. The Computer Vision, the Game Logic, and the Robot Control are all independent modules. This makes the system significantly easier to debug, test, and upgrade. 

Precise 2D-to-3D Coordinate Mapping: A key challenge is translating what the 2D camera "sees" into real-world 3D coordinates for the robot arm. This project implements a robust camera-to-robot calibration routine. This ensures that when the vision system identifies the center of a square at pixel $(x, y)$, the robot arm moves to the exact corresponding $(X, Y, Z)$ physical location to draw its symbol.

Closed-Loop Interaction: The system operates in an autonomous loop, making it highly interactive. It continuously monitors the board state with its camera. Once it detects that the human player has made a move, it automatically triggers its own decision-making and motion-planning sequence, creating a seamless, turn-based game experience without needing any button presses.

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

</div>
