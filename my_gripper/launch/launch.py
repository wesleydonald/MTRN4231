from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generates the launch description for the gripper control system.
    
    This launch file starts:
    1. The arduino_serial_node (Translator/Bridge)
    2. The end_effector_node (Manager/Logic)
    """

    # Node for the Arduino serial communication (Translator)
    arduino_serial_node = Node(
        package='my_gripper_pkg',
        executable='arduino_serial_node',
        name='arduino_serial_node',
        output='screen'  # Show print statements and logs in the terminal
    )

    # Node for the end-effector logic (Manager)
    end_effector_node = Node(
        package='my_gripper_pkg',
        executable='end_effector_node',
        name='end_effector_node',
        output='screen'
    )

    # Return the LaunchDescription object
    return LaunchDescription([
        arduino_serial_node,
        end_effector_node,
    ])