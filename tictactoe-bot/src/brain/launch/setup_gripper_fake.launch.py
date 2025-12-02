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
    ROBOT_IP = 'yyy.yyy.yyy.yyy'
    UR_TYPE = 'ur5e'
    USE_FAKE_HARDWARE = 'true'
    
    # Package and file names
    DRIVER_PACKAGE = 'ur_robot_driver'
    MOVEIT_PACKAGE = 'ur_moveit_config'
    END_EFFECTOR_PACKAGE = 'end_effector' # Or wherever your xacro is
    URDF_FILE = 'ur_with_gripper.xacro'
    
    # Delays for startup
    MOVEIT_START_DELAY = 1.0
    APP_START_DELAY = 3.0 # Give MoveIt time to start
    
    # --- 2. Find Custom Xacro (This is the "gripper visualization" part) ---
    urdf_file_path = PathJoinSubstitution([
        FindPackageShare(END_EFFECTOR_PACKAGE),
        "urdf",
        URDF_FILE
    ])

    description_params = {
        'ur_type': UR_TYPE
    }
    
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
            'ur_type': UR_TYPE,
            'robot_ip': ROBOT_IP,
            'use_fake_hardware': USE_FAKE_HARDWARE,
            'launch_rviz': 'false', # MoveIt will launch RViz

           
            # --- This is the key ---
            # Tell the driver to use YOUR gripper file
            'description_file': urdf_file_path,
        }.items()
    )

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
            'ur_type': UR_TYPE,
            'robot_ip': ROBOT_IP,
            'use_fake_hardware': USE_FAKE_HARDWARE,
            'launch_rviz': 'false', # Launch RViz
      
            # --- This is the FIX ---
            # Tell MoveIt to ALSO use your gripper file
            'description_file': urdf_file_path,
            'description_params': description_params,
            'launch_robot_state_publisher': 'false' # Driver handles this
        }.items()
    )
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
    # cv_launch_dir = PathJoinSubstitution([FindPackageShare('cv'), 'launch'])
    
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
        # IncludeLaunchDescription(PathJoinSubstitution([cv_launch_dir, 'cv_real.launch.py'])), 
        
        Node(
            package='visualization',
            namespace='',
            executable='visualization_node',
            name='visualization'
        ),
        Node(
            package='simulation',
            namespace='',
            executable='simulator',
            name='simulation'
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
        # Start the driver immediately
        ur_control_launch,
        
        # Start MoveIt after a 10s delay
        TimerAction(
            period=MOVEIT_START_DELAY,
            actions=[moveit_launch]
        ),
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