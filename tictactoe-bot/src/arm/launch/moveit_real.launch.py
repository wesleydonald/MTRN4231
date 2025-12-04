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
    MOVEIT_PACKAGE = 'ur_moveit_config'
    YOUR_PACKAGE_NAME = 'end_effector' # Or wherever your xacro is
    YOUR_URDF_FILE = 'ur_with_gripper.xacro'
    
    # Delays for startup
    MOVEIT_START_DELAY = 5.0
    
    # --- 2. Find Custom Xacro (This is the "gripper visualization" part) ---
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare(YOUR_PACKAGE_NAME),
        "urdf",
        YOUR_URDF_FILE
    ])

    description_params = {
        'ur_type': YOUR_UR_TYPE
    }
    
    # --- 4. Define MoveIt Launch (Now un-commented and configured) ---
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(MOVEIT_PACKAGE),
                'launch',
                'ur_moveit.launch.py'
            ])
        ),
        launch_arguments={
            'ur_type': YOUR_UR_TYPE,
            'robot_ip': YOUR_ROBOT_IP,
            'use_fake_hardware': USE_FAKE_HARDWARE,
            'launch_rviz': 'false', # Launch RViz
      
            # --- This is the FIX ---
            # Tell MoveIt to ALSO use your gripper file
            'description_file': urdf_file_path,
            'description_params': description_params,
            'launch_robot_state_publisher': 'false' # Driver handles this
        }.items()
    )

    # --- 6. Create the Final Launch Description ---
    return LaunchDescription([
        # Start MoveIt after a delay
        TimerAction(
            period=MOVEIT_START_DELAY,
            actions=[moveit_launch]
        ),
    ])
