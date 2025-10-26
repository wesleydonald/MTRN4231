import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():

    # --- Find all necessary files ---
    pkg_share = get_package_share_directory('my_gripper_description')
    
    urdf_file_path = os.path.join(
        pkg_share,
        'urdf',
        'my_gripper.xacro') # Make sure this is your xacro file name
    
    rviz_config_file = os.path.join(
        pkg_share, 
        'rviz', 
        'ur5e_with_gripper.rviz') # Make sure this is your rviz file name

    # --- Process the XACRO file ---
    doc = xacro.parse(open(urdf_file_path))
    xacro.process_doc(doc)
    robot_description = doc.toxml()
    # -------------------------------


    # 1. Node: robot_state_publisher
    # (This node is unchanged)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. Node: joint_state_publisher (PDF Task 5 Method)
    # This replaces the GUI and your custom controller.
    # It will listen for messages on topics defined in 'source_list'
    # and merge them into the main /joint_states topic.
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'source_list': ['left_claw_joint'] 
            # We only need the main joint, since the other mimics it.
            # This will make the node listen on the /left_claw_joint topic.
        }]
    )

    # 3. Node: RViz2
    # (This node is unchanged)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    # 4. Node: 我们的新控制器！(REMOVED)
    # We remove this node because its job is now done by
    # publishing from the command line (Step 3).
    #
    # gripper_controller_node = Node(...)


    # Add all nodes to the launch description
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node, # <-- Added this node
        rviz_node
        # gripper_controller_node <-- Removed this node
    ])

