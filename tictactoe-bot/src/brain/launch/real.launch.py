import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node 
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Delays for startup
    APP_START_DELAY = 10.0 # Give MoveIt time to start
    
    rviz_path = 'rviz_config/rviz.rviz'
    rviz_file = os.path.join(get_package_share_directory('brain'), rviz_path)
    rviz_launch = Node(
        name='rviz2',
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_file]
    )

    # --- 5. Define Your Application Nodes (Originals from cv_real.launch.py) ---
    
    # Find your package locations
    arm_launch_dir = PathJoinSubstitution([FindPackageShare('arm'), 'launch'])
    cv_launch_dir = PathJoinSubstitution([FindPackageShare('cv'), 'launch'])
    
    # Create a list of all your application nodes
    application_nodes = [
        IncludeLaunchDescription(PathJoinSubstitution([arm_launch_dir, 'arm.launch.py'])),
        
        # !! POTENTIAL ERROR FIX !!
        # Your original file 'cv_real.launch.py' was launching itself.
        # Now that this file is named 'setup_gripper.launch.py',
        # you might want to include the 'cv_real.launch.py' file here *if*
        # it contains your camera/CV nodes.
        #
        # If your CV/camera launch file is named something else,
        # like 'camera.launch.py', change it below.
        IncludeLaunchDescription(PathJoinSubstitution([cv_launch_dir, 'cv_real.launch.py'])), 
        
        Node(
            package='visualization',
            namespace='',
            executable='visualization_node',
            name='visualization'
        ),
        Node(
            package='brain',
            namespace='',
            executable='brain',
            name='brain'
        ),
        Node(
            package='end_effector',
            namespace='',
            executable='gripper',
            name='gripper'
        )
    ]

    # --- 6. Create the Final Launch Description ---
    return LaunchDescription([
        TimerAction(
            period=APP_START_DELAY,
            actions=[rviz_launch]
        ),
        # Start all your application nodes after a 15s delay
        TimerAction(
            period=APP_START_DELAY,
            actions=application_nodes
        )
    ])
