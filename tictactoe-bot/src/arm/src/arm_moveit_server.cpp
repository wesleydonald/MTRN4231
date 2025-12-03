#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <interfaces/srv/move_arm.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/orientation_constraint.h>
#include <moveit_msgs/msg/joint_constraint.h>
#include <moveit_msgs/msg/constraints.h>
#include "std_srvs/srv/trigger.hpp" 

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

constexpr double PLANNING_TIME = 0.1;
constexpr int PLANNING_ATTEMPTS = 5;

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

struct JointConstraintConfig {
  std::string joint_name;
  double position;
  double tolerance_above;
  double tolerance_below;
};

const std::vector<JointConstraintConfig> JOINT_CONSTRAINTS = {
    { "shoulder_pan_joint",   0,          M_PI / 2,     M_PI / 1.5 },
    { "shoulder_lift_joint",  -M_PI / 2,  M_PI /2,      M_PI /2},
    // { "elbow_joint",       0,          M_PI,         M_PI },
    { "wrist_1_joint",        -M_PI/2,    M_PI*4/5,     M_PI*4/5},
    { "wrist_2_joint",        -M_PI/2,    M_PI/1.5,     M_PI/1.5 },
    { "wrist_3_joint",        0,   M_PI / 1.5,   M_PI / 1.5 }
};

const std::vector<double> HOME_JOINT_CONFIG = { 
  0.0,                    // 0    deg   shoulder_pan
  -90.0*(M_PI/180.0),     // -90  deg   shoulder_lift
  90.0*(M_PI/180.0),      // 90   deg   elbow_joint
  -M_PI/2.0,              // -90  deg   wrist_1_joint
  -M_PI/2.0,              // -90  deg   wrist_2_joint
  0.0                     // 0    deg   wrist_3_joint
};

using std::placeholders::_1;

class arm : public rclcpp::Node {
public:
    arm() : Node("arm"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)  {
      move_request_ = create_service<interfaces::srv::MoveArm>("arm_service", std::bind(&arm::moveCallback, this, std::placeholders::_1, std::placeholders::_2));
      stop_service_ = create_service<std_srvs::srv::Trigger>("arm_stop_service", std::bind(&arm::stopCallback, this, std::placeholders::_1, std::placeholders::_2));
      
      move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::shared_ptr<rclcpp::Node>(this), "ur_manipulator");
      move_group_interface->setPlanningTime(PLANNING_TIME);
      move_group_interface->setNumPlanningAttempts(PLANNING_ATTEMPTS);
      move_group_interface->setPlannerId("TRRTkConfigDefault");
      //move_group_interface->setPlannerId("RRTConnectkConfigDefault");
      // move_group_interface->setPlannerId("RRTConnect");
      //move_group_interface->setPlannerId("RRTstarkConfigDefault");

      std::string frame_id = move_group_interface->getPlanningFrame();

      // Generate the objects to avoid
      moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
      planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 0.04, 1.0, 0.85, -0.30, 0.5, frame_id, "backWall"));
      planning_scene_interface.applyCollisionObject(generateCollisionObject(0.04, 1.2, 1.0, -0.30, 0.25, 0.5, frame_id, "sideWall"));
      planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.01, 0.85, 0.25, 0.013, frame_id, "table"));
      // planning_scene_interface.applyCollisionObject(generateCollisionObject(2.4, 2.4, 0.04, 0.85, 0.25, 1.0, frame_id, "ceiling"));
    }

private:
  void moveCallback(const std::shared_ptr<interfaces::srv::MoveArm::Request> request,
           std::shared_ptr<interfaces::srv::MoveArm::Response> response) {
    move_group_interface->setStartStateToCurrentState();
    move_group_interface->setMaxVelocityScalingFactor(1.0);
    move_group_interface->setMaxAccelerationScalingFactor(1.0);

    if (request->move_home) {
      response->success = moveHome();
      response->message = "success";
      return;
    }

    moveit_msgs::msg::Constraints constraints;
    set_joint_constraints(constraints);
    move_group_interface->setPathConstraints(constraints);
    std::vector<geometry_msgs::msg::Pose> waypoints;

    geometry_msgs::msg::Pose target_pose = request->target_pose;
    // target_pose.position.x += 0.02;
    // target_pose.position.y -= 0.01;
    target_pose.position.z = 0.32;
    target_pose.orientation.x = - std::sqrt(2) / 2;
    target_pose.orientation.y = std::sqrt(2) / 2;
    target_pose.orientation.z = 0.0;
    target_pose.orientation.w = 0.0;

    geometry_msgs::msg::Pose current_pose = target_pose;
    auto tf = tf_buffer_.lookupTransform("base_link", "tool0", tf2::TimePointZero);
    current_pose.position.x = tf.transform.translation.x;
    current_pose.position.y = tf.transform.translation.y;
    current_pose.position.z = tf.transform.translation.z;
    
    waypoints.push_back(current_pose);
    if (current_pose.position.z < 0.3 && current_pose.position.z > 0.0) {
      current_pose.position.z = 0.32;
      waypoints.push_back(current_pose);
    }

    waypoints.push_back(target_pose);
    target_pose.position.z = 0.24;
    waypoints.push_back(target_pose);

    const double eef_step = 0.01;
    const double jump_threshold = 0.0;

    moveit_msgs::msg::RobotTrajectory trajectory;
    double fraction = move_group_interface->computeCartesianPath(
        waypoints,
        eef_step,
        jump_threshold,
        trajectory,
        true
    );

    RCLCPP_INFO(this->get_logger(), "Cartesian path fraction: %.2f%%", fraction * 100.0);

    bool success = (fraction > 0.95);

    if (!success) {
        RCLCPP_WARN(this->get_logger(), "Failed to compute a sufficient Cartesian path (%.2f).", fraction);
        move_group_interface->clearPathConstraints();
        response->success = false;
        response->message = "failed to plan";
        return;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    plan.trajectory_ = trajectory;
    move_group_interface->execute(plan);
    move_group_interface->clearPathConstraints();
    response->success = true;
    response->message = "success";
  }

  bool moveHome() {
    move_group_interface->setJointValueTarget(HOME_JOINT_CONFIG);
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;

    if (move_group_interface->plan(home_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_interface->execute(home_plan);
        return true;
    }
    return false;
  }

  // --- 3. ADDED THIS ENTIRE CALLBACK FUNCTION ---
  void stopCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
      (void)request; // To prevent unused parameter warning
      RCLCPP_ERROR(this->get_logger(), "!!! E-STOP SERVICE CALLED - STOPPING ARM !!!");
      
      // This is the MoveIt command to stop all current motion
      move_group_interface->stop();

      response->success = true;
      response->message = "Arm motion stopped.";
  }

  void set_joint_constraints(moveit_msgs::msg::Constraints &constraints) {
    for (const auto& config : JOINT_CONSTRAINTS) {
      moveit_msgs::msg::JointConstraint constraint;
      constraint.joint_name = config.joint_name;
      constraint.position = config.position;
      constraint.tolerance_above = config.tolerance_above;
      constraint.tolerance_below = config.tolerance_below;
      constraint.weight = 1.0;
      constraints.joint_constraints.push_back(constraint);
    }
  }

  void set_orientation_constraints(moveit_msgs::msg::Constraints &constraints) {
    moveit_msgs::msg::OrientationConstraint orientation_constraint;

    orientation_constraint.header.frame_id = move_group_interface->getPlanningFrame();
    orientation_constraint.link_name = move_group_interface->getEndEffectorLink();

    // Absolute tolerances in RADIANS
    orientation_constraint.absolute_x_axis_tolerance = 0.4;
    orientation_constraint.absolute_y_axis_tolerance = 0.4; 
    orientation_constraint.absolute_z_axis_tolerance = 3.14; 

    orientation_constraint.weight = 1.0;

    // tf2::Quaternion q;
    // q.setRPY(M_PI, 0, 0);
    // auto current_pose = move_group_interface->getCurrentPose();
    orientation_constraint.orientation.x = - std::sqrt(2) / 2;
    orientation_constraint.orientation.y = std::sqrt(2) / 2;
    orientation_constraint.orientation.z = 0.0;
    orientation_constraint.orientation.w = 0.0;

    constraints.orientation_constraints.emplace_back(orientation_constraint);
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
  rclcpp::Service<interfaces::srv::MoveArm>::SharedPtr move_request_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<arm>());
  rclcpp::shutdown();
  return 0;
}
