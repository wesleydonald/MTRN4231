import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseArray, PoseStamped
from moveit_msgs.msg import RobotState
from moveit_msgs.action import MoveGroup
from tf2_ros import Buffer, TransformListener
import time

class ArmMover(Node):
    def __init__(self):
        super().__init__('arm_mover')
        
        # Subscriber to the pieces topic
        self.subscription = self.create_subscription(
            PoseArray,
            '/object_recognizer/detected/pieces',
            self.piece_callback,
            10)
        
        # Action client for MoveGroup
        self._move_group_client = ActionClient(self, MoveGroup, '/move_action')
        
        self.get_logger().info("Arm Mover node has started. Waiting for piece detections...")
        self.target_received = False

    def piece_callback(self, msg):
        # Only process the first message received to avoid constant re-planning
        if not self.target_received and len(msg.poses) > 0:
            self.target_received = True
            
            target_pose = msg.poses[0] # Pick the first detected piece
            self.get_logger().info(f"Received piece location: {target_pose.position}")
            
            # Add a vertical offset to approach from above
            target_pose.position.z += 0.15 # 15cm above the piece
            
            self.move_to_target(target_pose)

    def move_to_target(self, pose):
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._move_group_client.wait_for_server()
        
        goal_msg = MoveGroup.Goal()
        
        # Set up the motion planning request
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.pose = pose
        
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.goal_constraints.append(
            self.create_position_constraint(pose_stamped)
        )
        goal_msg.request.allowed_planning_time = 5.0
        
        self.get_logger().info("Sending goal to MoveGroup...")
        self._send_goal_future = self._move_group_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def create_position_constraint(self, pose_stamped):
        from moveit_msgs.msg import Constraints, PositionConstraint
        constraint = Constraints()
        p_constraint = PositionConstraint()
        p_constraint.header.frame_id = pose_stamped.header.frame_id
        p_constraint.link_name = "tool0" # The name of your end-effector link
        p_constraint.target_point_offset.x = 0.0
        p_constraint.target_point_offset.y = 0.0
        p_constraint.target_point_offset.z = 0.0
        
        # Bounding box for the target position
        from shape_msgs.msg import SolidPrimitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01] # 1cm tolerance box
        
        from geometry_msgs.msg import Pose
        constraint_pose = Pose()
        constraint_pose.position = pose_stamped.pose.position
        
        p_constraint.constraint_region.primitives.append(box)
        p_constraint.constraint_region.primitive_poses.append(constraint_pose)
        p_constraint.weight = 1.0
        
        constraint.position_constraints.append(p_constraint)
        return constraint

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by MoveGroup")
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
            
        # Allow the node to process another target
        self.target_received = False

def main(args=None):
    rclpy.init(args=args)
    node = ArmMover()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
