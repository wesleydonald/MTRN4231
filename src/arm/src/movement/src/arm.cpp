#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

constexpr double PLANNING_TIME = 0.1;
constexpr int PLANNING_ATTEMPTS = 500;

//Function to generate a collision object
auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

//Function to generate a target position message
auto generatePoseMsg(float x,float y, float z,float qx,float qy,float qz,float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

auto generateAttachedEECollisionObject(
    float sx, float sy, float sz, 
    float x, float y, float z, 
    std::string ee_link, std::string id) 
{
    moveit_msgs::msg::AttachedCollisionObject attached_object;

    attached_object.link_name = ee_link;
    attached_object.object = generateCollisionObject(sx, sy, sz, x, y, z, ee_link, id);

    return attached_object;
}

using std::placeholders::_1;

class arm : public rclcpp::Node {
public:
    arm() : Node("arm") {

      // Initalise the transformation listener
      tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

      // Look up the transformation ever 200 milliseconds
      timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&arm::tfCallback, this));

      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
      move_group_interface->setPlanningTime(PLANNING_TIME);
      move_group_interface->setNumPlanningAttempts(PLANNING_ATTEMPTS);
      // move_group_interface->setPlannerId("RRTConnectkConfigDefault");
      // move_group_interface->setPlannerId("RRTConnect");
      //move_group_interface->setPlannerId("BKPIECEkConfigDefault");
      move_group_interface->setPlannerId("TRRTkConfigDefault");
      //move_group_interface->setPlannerId("RRTstarkConfigDefault");

      std::string frame_id = move_group_interface->getPlanningFrame();

      // Generate the objects to avoid
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      
      // Apply collision objects
      planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
      planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
      planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.01, 0.85, 0.25, 0.013, frame_id, "table"));
      planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.0, frame_id, "ceiling"));
      
      moveit_msgs::msg::AttachedCollisionObject attached_object;
      attached_object.link_name = move_group_interface->getEndEffectorLink();
      attached_object.object = generateCollisionObject(
          0.1, 0.1, 0.15,
          0.0, 0.0, 0.10,
          attached_object.link_name,
          "end_effector"
      );
      planning_scene_interface.applyAttachedCollisionObject(attached_object);

      moveToPose();
    }

private:

  void moveToPose() {
    auto current_pose = move_group_interface->getCurrentPose();

    auto target_pose = generatePoseMsg(
       0.4, 0.2, 0.3,
       1.0, 0.0, 0.0, 0.0 
    );
    //auto target_pose = current_pose;
    //target_pose.pose.position.x = 0.4;
    //target_pose.pose.position.y = 0.2;
    //target_pose.pose.position.z = 0.3;

    // Plan to that pose
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    move_group_interface->setStartStateToCurrentState();
    move_group_interface->setPoseTarget(target_pose, "tool0");
    moveit_msgs::msg::Constraints constraints = set_constraint();
    move_group_interface->setPathConstraints(constraints);

    bool success = false;
    double planningTime = 0.1;
    const double maxPlanningTime = 60.0;

    while (!success && planningTime <= maxPlanningTime) {
        move_group_interface->setPlanningTime(planningTime);
        RCLCPP_INFO(this->get_logger(), "Trying to plan with %.1f seconds...", planningTime);
        success = (move_group_interface->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if (!success) {
            planningTime *= 2;
        }
    }

    if (success) {
      RCLCPP_INFO(this->get_logger(), "Planning successful, executing...");
      move_group_interface->execute(plan);
    } else {
      RCLCPP_WARN(this->get_logger(), "Planning failed.");
    }
  }

  moveit_msgs::msg::Constraints set_constraint() { 
    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.emplace_back(set_orientation_constraint());
    //constraints.joint_constraints.emplace_back(set_joint_constraint());
    return constraints;
  }

  moveit_msgs::msg::OrientationConstraint set_orientation_constraint() {
    moveit_msgs::msg::OrientationConstraint orientation_constraint;

    orientation_constraint.header.frame_id = move_group_interface->getPlanningFrame();
    orientation_constraint.link_name = move_group_interface->getEndEffectorLink();

    // Absolute tolerances in radians
    orientation_constraint.absolute_x_axis_tolerance = 0.4; // ~3°
    orientation_constraint.absolute_y_axis_tolerance = 0.4; 
    orientation_constraint.absolute_z_axis_tolerance = 3.14; // allow free rotation around Z

    orientation_constraint.weight = 1.0;

    // Straight down quaternion (tool Z pointing down)
    tf2::Quaternion q;
    q.setRPY(M_PI, 0, 0); // flip X-axis by 180° if your tool frame Z points forward
    orientation_constraint.orientation.x = q.x();
    orientation_constraint.orientation.y = q.y();
    orientation_constraint.orientation.z = q.z();
    orientation_constraint.orientation.w = q.w();

    return orientation_constraint;
  }


  moveit_msgs::msg::JointConstraint set_joint_constraint() {
    moveit_msgs::msg::JointConstraint elbow_constraint;
    elbow_constraint.joint_name = "elbow_joint";
    // Convert degrees to radians
    const double min_angle = 60.0 * M_PI / 180.0; // 60° in radians (~1.0472)
    const double max_angle = 150.0 * M_PI / 180.0; // 150° in radians (~2.6179)
    // Calculate midpoint (105° which is between 60° and 150°)
    const double midpoint = (min_angle + max_angle) / 2.0; // ~1.8326 radians
    // Set constraints
    elbow_constraint.position = midpoint;
    elbow_constraint.tolerance_below = 1.0; // ~0.7854 radians (45°)
    elbow_constraint.tolerance_above = max_angle - midpoint; // ~0.7854 radians (45°)
    elbow_constraint.weight = 1.0;
    RCLCPP_INFO(this->get_logger(), "elbow constraint implemented.");
    return elbow_constraint;
  }

  void tfCallback() {
    // Check if the transformation is between "ball_frame" and "base_link" 
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm>());
  rclcpp::shutdown();
  return 0;
}
