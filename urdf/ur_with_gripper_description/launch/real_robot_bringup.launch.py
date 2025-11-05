import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction, IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # --- 1. Configuration ---
    # (You can customize these values)
    YOUR_ROBOT_IP = '192.168.0.100'
    YOUR_UR_TYPE = 'ur5e'
    DELAY_SECONDS = 10.0

    # !! Set to 'true' since you are not connected to the real robot !!
    USE_FAKE_HARDWARE = 'false'

    # Package and file names (these should be correct)
    DRIVER_PACKAGE = 'ur_robot_driver'
    MOVEIT_PACKAGE = 'ur_moveit_config'
    YOUR_PACKAGE_NAME = 'ur_with_gripper_description'
    YOUR_URDF_FILE = 'ur_with_gripper.xacro'

    # --- 2. Find Your Custom Xacro File ---
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare(YOUR_PACKAGE_NAME),
        "urdf",
        YOUR_URDF_FILE
    ])

    # --- 3. Define Xacro Parameters ---
    # These parameters will be passed to your xacro file
    description_params = {
        'ur_type': YOUR_UR_TYPE
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
        launch_arguments={
            'ur_type': YOUR_UR_TYPE,
            'robot_ip': YOUR_ROBOT_IP,
            'use_fake_hardware': USE_FAKE_HARDWARE,
            'launch_rviz': 'false',
            
            # --- This is the key ---
            # Tell the driver to use YOUR gripper file
            'description_file': urdf_file_path,
            'description_params': description_params,
            
            # Let the driver launch the (correct) Robot State Publisher
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
            'ur_type': YOUR_UR_TYPE,
            'robot_ip': YOUR_ROBOT_IP,
            'use_fake_hardware': USE_FAKE_HARDWARE,
            'launch_rviz': 'true',
            
            # --- This is the FIX ---
            # Tell MoveIt to ALSO use your gripper file for its internal setup
            'description_file': urdf_file_path,
            'description_params': description_params,
            
            # Tell MoveIt NOT to launch a second, conflicting RSP
            'launch_robot_state_publisher': 'false' 
        }.items()
    )

    # --- 6. Create the Final Launch Description ---
    return LaunchDescription([
        ur_control_launch,
        
        TimerAction(
            period=DELAY_SECONDS,
            actions=[moveit_launch]
        )
    ])