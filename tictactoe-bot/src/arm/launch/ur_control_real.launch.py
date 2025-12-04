import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- 1. Configuration (from real_robot_bringup.py) ---
    YOUR_ROBOT_IP = '192.168.0.100'
    YOUR_UR_TYPE = 'ur5e'
    USE_FAKE_HARDWARE = 'false'
    
    # Package and file names
    DRIVER_PACKAGE = 'ur_robot_driver'
    YOUR_PACKAGE_NAME = 'end_effector' # Or wherever your xacro is
    YOUR_URDF_FILE = 'ur_with_gripper.xacro'
    
    # --- 2. Find Custom Xacro (This is the "gripper visualization" part) ---
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare(YOUR_PACKAGE_NAME),
        "urdf",
        YOUR_URDF_FILE
    ])

    # --- 3. Define UR Robot Driver Launch (Now un-commented and configured) ---
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(DRIVER_PACKAGE),
                'launch',
                'ur_control.launch.py' 
            ])
        ),
        launch_arguments={
            'ur_type': YOUR_UR_TYPE,
            'robot_ip': YOUR_ROBOT_IP,
            'use_fake_hardware': USE_FAKE_HARDWARE,
            'launch_rviz': 'false', # MoveIt will launch RViz
            'description_file': urdf_file_path,
        }.items()
    )

    return LaunchDescription([
        # Start the driver immediately
        ur_control_launch,
    ])
