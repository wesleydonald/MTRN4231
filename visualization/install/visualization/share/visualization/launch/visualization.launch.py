import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    # --- 1. Configuration ---
    YOUR_ROBOT_IP = '192.168.0.100'
    YOUR_UR_TYPE = 'ur5e'
    DELAY_MOVEIT_SECONDS = 5.0  # Time to wait for driver to start
    DELAY_GRIPPER_SECONDS = 10.0 # Time to wait for MoveIt to start

    # --- 2. Package Names (DEFAULT PACKAGES) ---
    DRIVER_PACKAGE = 'ur_robot_driver'
    MOVEIT_PACKAGE = 'ur_moveit_config' # The default, pre-built package
    VISUALIZATION_PACKAGE = 'visualization' # Your new package

    # --- 3. Define Parameters ---
    # We pass these to the included launch files
    # They will use these to find all the correct config files
    launch_params = {
        'ur_type': YOUR_UR_TYPE,
        'robot_ip': YOUR_ROBOT_IP,
        'use_fake_hardware': 'false', # Set to 'true' if not using real robot
    }

    # --- 4. Define the UR Robot Driver Launch ---
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(DRIVER_PACKAGE),
                'launch',
                'ur_control.launch.py' 
            ])
        ),
        # Pass parameters to the included launch file
        launch_arguments={
            'ur_type': launch_params['ur_type'],
            'robot_ip': launch_params['robot_ip'],
            'use_fake_hardware': launch_params['use_fake_hardware'],
            'launch_rviz': 'false', # We let MoveIt launch RViz
            # 'description_file' REMOVED - This was the bug
            'launch_robot_state_publisher': 'true'
        }.items()
    )

    # --- 5. Define the MoveIt Launch ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(MOVEIT_PACKAGE),
                'launch',
                'ur_moveit.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': launch_params['ur_type'],
            'robot_ip': launch_params['robot_ip'],
            'use_fake_hardware': launch_params['use_fake_hardware'],
            'launch_rviz': 'true',
            # 'description_file' REMOVED - This was the bug
            'launch_robot_state_publisher': 'false' # Driver already launched one
        }.items()
    )
    
    # --- 6. Define the Gripper Attacher Node ---
    attach_gripper_node = Node(
        package=VISUALIZATION_PACKAGE,
        executable='attach_gripper', # The name from your setup.py
        name='attach_gripper_node',
        output='screen'
    )

    # --- 7. Create the Final Launch Description ---
    return LaunchDescription([
        ur_control_launch,
        
        TimerAction(
            period=DELAY_MOVEIT_SECONDS,
            actions=[moveit_launch]
        ),
        
        TimerAction(
            period=DELAY_GRIPPER_SECONDS,
            actions=[attach_gripper_node]
        )
    ])