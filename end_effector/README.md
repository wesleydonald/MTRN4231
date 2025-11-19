# ROS 2 Arduino Gripper Project

This project contains two ROS 2 packages to control a simple servo-based gripper connected to an Arduino.

## Packages

### my_gripper_interfaces : 
Defines the custom service  that the "brain" node uses to send a command.

### my_gripper_pkg : 
Contains the two Python nodes:

  *end_effector_node : Provides the  service and translates commands into a simple topic.

  *arduino_serial_node : Subscribes to the topic and sends the command over the USB serial port to the Arduino.



## 1. How to Run

1. **Plug in your Arduino.**

2. **Find your Arduino's port name.** In a terminal, run:

   ```
   ls /dev/tty*
   ```

   Look for a port like /dev/ttyACM0 or /dev/ttyUSB0.

3. **Update the code (if needed).** Open my_gripper_pkg/my_gripper_pkg/arduino_serial_node.py and make sure the usb_port variable matches the port you found.

4. **Source your workspace.** You must do this in every new terminal you open.

   ```
   # Make sure to be in your workspace root folder
   source install/setup.bash
   ```

5. **Run the launch file.** This will start both Python nodes at once.

   ```
   ros2 launch my_gripper_pkg gripper.launch.py
   ```

## 2. How to Test

With the launch file running in one terminal, open a **second terminal**.

1. **Source your workspace** in the new terminal:

   ```
   source install/setup.bash
   ```

2. **Call the service** to send a "GRASP" command:

   ```
   ros2 service call /end_effector_srv my_gripper_interfaces/srv/EndEffectorCmd '{command: "GRASP"}'
   ```

3. **Call the service** to send a "RELEASE" command:

   ```
   ros2 service call /end_effector_srv my_gripper_interfaces/srv/EndEffTCMd '{command: "RELEASE"}'
   ```

You should see the logs in your first terminal and (if your Arduino is plugged in and programmed) see your physical gripper move.
