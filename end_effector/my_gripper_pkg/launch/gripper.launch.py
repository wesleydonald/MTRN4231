import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # Node for the end_effector service
    end_effector_node = Node(
        package='my_gripper_pkg',
        executable='end_effector_node',
        name='end_effector_node',
        output='screen'
    )

    # Node for the Arduino serial communication
    arduino_serial_node = Node(
        package='my_gripper_pkg',
        executable='arduino_serial_node',
        name='arduino_serial_node',
        output='screen'  # Print directly to the console
    )

    # Add both nodes to the launch description
    return LaunchDescription([
        end_effector_node,
        arduino_serial_node,
    ])