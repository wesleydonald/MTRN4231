#!/usr/bin/env python3
import sys
import rclpy
import time
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose, PoseStamped
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import (
    PlanningScene,
    CollisionObject,
    Constraints,
    PositionConstraint,
    OrientationConstraint
)
from moveit_msgs.action import MoveGroup
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import rclpy.time

class ArmMover(Node):
    """
    A ROS 2 node to move the UR5e robot arm to a detected TF frame.

    This node uses the low-level ActionClient (no moveit_commander) to
    re-implement the logic from 'move_to_marker.cpp'.
    It publishes collision objects, listens to TF, and sends goals
    to the /move_action server.
    """

    def __init__(self):
        super().__init__('arm_mover')
        self.get_logger().info("Initializing ArmMover node (ActionClient version)...")

        # Flag to prevent concurrent movements
        self.is_moving = False
        self.planning_frame = "base_link" # Assume planning frame

        # Frame to look for (from object_recognizer.py)
        self.target_frame = 'chessboard'
        # Note: you can change this to 'white_piece_0', 'black_piece_0', etc.

        try:
            # Action client for MoveGroup
            self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
            
            # Publisher for adding/removing collision objects
            self.planning_scene_diff_publisher = self.create_publisher(
                PlanningScene, '/planning_scene', 10
            )

            # TF Listener (learning from move_to_marker.cpp and lab PDF)
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

            # Wait for publishers/servers
            time.sleep(2.0)

            # Add collision objects (manually, without moveit_commander)
            self.add_collision_objects()

            # Create a timer to look up the transform
            self.timer = self.create_timer(1.0, self.tf_callback)
            
            self.get_logger().info(f"ArmMover node initialized. Listening for TF frame '{self.target_frame}'.")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize: {e}")
            rclpy.shutdown()

    def _create_collision_box(self, box_id, frame_id, dimensions, position):
        """Helper function to create a CollisionObject msg."""
        collision_object = CollisionObject()
        collision_object.id = box_id
        collision_object.header.frame_id = frame_id
        
        primitive = SolidPrimitive()
        primitive.type = primitive.BOX
        primitive.dimensions = list(dimensions) # e.g., [2.4, 1.0, 0.05]

        box_pose = Pose()
        box_pose.position.x = position[0]
        box_pose.position.y = position[1]
        box_pose.position.z = position[2]
        box_pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(box_pose)
        collision_object.operation = collision_object.ADD
        return collision_object

    def add_collision_objects(self):
        """
        Manually builds and publishes CollisionObjects to the /planning_scene.
        """
        self.get_logger().info("Adding collision objects (table, walls)...")
        
        # 1. Table
        table = self._create_collision_box(
            "table", self.planning_frame,
            (2.4, 1.0, 0.05), (0.85, 0.25, 0.05)
        )
        # 2. Back Wall
        backWall = self._create_collision_box(
            "backWall", self.planning_frame,
            (2.4, 0.04, 1.0), (0.85, -0.30, 0.5)
        )
        # 3. Side Wall
        sideWall = self._create_collision_box(
            "sideWall", self.planning_frame,
            (0.04, 1.2, 1.0), (-0.30, 0.25, 0.5)
        )

        # Create a PlanningScene message
        planning_scene_msg = PlanningScene()
        planning_scene_msg.world.collision_objects.append(table)
        planning_scene_msg.world.collision_objects.append(backWall)
        planning_scene_msg.world.collision_objects.append(sideWall)
        planning_scene_msg.is_diff = True # Mark this as a diff

        # Publish the collision objects
        self.planning_scene_diff_publisher.publish(planning_scene_msg)
        
        # Short delay to allow the planning scene to update
        time.sleep(1.0)
        self.get_logger().info("Collision objects published.")


    def tf_callback(self):
        """
        Timer callback to look up the target frame and plan a move.
        """
        if self.is_moving:
            # self.get_logger().info("Already moving, skipping TF lookup.")
            return

        try:
            # Look up the transform from the planning frame to the target frame
            t = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.target_frame,
                rclpy.time.Time())
        
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f"Could not find transform '{self.target_frame}': {e}")
            return

        self.is_moving = True
        self.get_logger().info(f"Found target '{self.target_frame}'. Moving to position.")

        target_pose = Pose()
        
        # Use the X and Y from the TF lookup
        target_pose.position.x = t.transform.translation.x
        target_pose.position.y = t.transform.translation.y
        
        # IGNORE the detected Z-value. (Fix for GOAL_IN_COLLISION error)
        table_top_z = 0.075 # From (0.05 center + 0.05/2 height)
        hover_offset = 0.15
        target_pose.position.z = table_top_z + hover_offset # This will be 0.175
        
        self.get_logger().info(f"Targeting X={target_pose.position.x:.3f}, Y={target_pose.position.y:.3f}, Z={target_pose.position.z:.3f}")

        # Set orientation to "pointing down" (Roll=pi)
        target_pose.orientation.x = 1.0
        target_pose.orientation.y = 0.0
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0

        # Send the goal using the ActionClient
        self.move_to_target(target_pose)

    def move_to_target(self, pose):
        """Builds and sends a MoveGroup.Goal via the ActionClient."""
        
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._move_group_client.wait_for_server()
        
        goal_msg = MoveGroup.Goal()
        
        # --- Configure the Request ---
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.allowed_planning_time = 10.0 # Fix for TIMED_OUT
        goal_msg.request.num_planning_attempts = 3
        goal_msg.request.planner_id = "RRTConnect" # A good default planner

        # --- FIX FOR "exceeding joint velocity" ERROR ---
        # Scale down the max velocity and acceleration to 20% of the robot's limit
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2
        # --- END OF FIX ---

        # --- Create PoseStamped for the target ---
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.pose = pose

        # --- Create Position Constraint ---
        p_constraint = PositionConstraint()
        p_constraint.header.frame_id = pose_stamped.header.frame_id
        p_constraint.link_name = "tool0" # End-effector link
        p_constraint.target_point_offset.x = 0.0
        p_constraint.target_point_offset.y = 0.0
        p_constraint.target_point_offset.z = 0.0
        # A small tolerance box
        box_primitive = SolidPrimitive()
        box_primitive.type = box_primitive.BOX
        box_primitive.dimensions = [0.02, 0.02, 0.02] # 1cm tolerance
        p_constraint.constraint_region.primitives.append(box_primitive)
        p_constraint.constraint_region.primitive_poses.append(pose_stamped.pose)
        p_constraint.weight = 1.0

        # --- Create Orientation Constraint ---
        o_constraint = OrientationConstraint()
        o_constraint.header.frame_id = pose_stamped.header.frame_id
        o_constraint.link_name = "tool0"
        o_constraint.orientation = pose_stamped.pose.orientation
        o_constraint.absolute_x_axis_tolerance = 0.5
        o_constraint.absolute_y_axis_tolerance = 0.5
        o_constraint.absolute_z_axis_tolerance = 3.14 # Allow full rotation around tool Z
        o_constraint.weight = 1.0

        # --- Add constraints to the goal ---
        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(p_constraint)
        goal_constraints.orientation_constraints.append(o_constraint)
        goal_msg.request.goal_constraints.append(goal_constraints)
        
        # --- Send the Goal ---
        self.get_logger().info("Sending goal to MoveGroup (ActionClient)...")
        self._send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
            self.is_moving = False # Reset flag
            return
            
        self.get_logger().info("Goal accepted. Waiting for result...")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code.val == 1: # SUCCESS
            self.get_logger().info("Movement successful!")
        else:
            self.get_logger().error(f"Movement failed with error code: {result.error_code.val}")
            
        # Allow for new movements on the next timer tick
        self.is_moving = False

    def on_shutdown(self):
        """
        Cleanup on node shutdown.
        """
        self.get_logger().info("Shutting down...")
        if hasattr(self, 'planning_scene_diff_publisher'):
            # Publish a scene diff to remove the objects
            planning_scene_msg = PlanningScene()
            
            # Create empty collision objects with REMOVE operation
            table = CollisionObject()
            table.id = "table"
            table.operation = table.REMOVE
            
            backWall = CollisionObject()
            backWall.id = "backWall"
            backWall.operation = backWall.REMOVE

            sideWall = CollisionObject()
            sideWall.id = "sideWall"
            sideWall.operation = sideWall.REMOVE

            planning_scene_msg.world.collision_objects.extend([table, backWall, sideWall])
            planning_scene_msg.is_diff = True
            self.planning_scene_diff_publisher.publish(planning_scene_msg)
            self.get_logger().info("Collision objects removed.")
            time.sleep(1.0) # Give time to publish


def main(args=None):
    rclpy.init(args=args)
    
    arm_mover_node = None
    try:
        arm_mover_node = ArmMover()
        rclpy.spin(arm_mover_node)
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        rclpy.logging.get_logger("main").error(f"Unhandled exception: {e}")
    finally:
        if arm_mover_node is not None and rclpy.ok():
            arm_mover_node.on_shutdown()
            arm_mover_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
