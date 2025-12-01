/*
SIMULATOR NODE -> Publishes piece and board locations when away from the lab
                  used for testing other packages.

PUBLISHES TO   -> /detected/board
               -> /detected/white/pieces
               -> /detected/black/pieces

The idea is we are in the PLAYER_TURN state until we see a piece has moved
onto the board. Then we calculate the best move. We then enter the ROBOT_TURN state
where we send a service request to the arm to move to a certain x, y 
location. When we recieve the completed response from the arm node we
send a service request to the gripper node to close the gripper. Then 
a similar thing for lifting the piece and placing it on the board. 
Lastly we return to home_pose.  
*/

#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "interfaces/msg/board_pose.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

class Simulator : public rclcpp::Node {
public:
  Simulator() : Node("simulator") {
    // Publishers
    board_pub_ = this->create_publisher<interfaces::msg::BoardPose>("/detected/board", 10);
    white_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/pieces/white", 10);
    black_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/pieces/black", 10);

    // Timer
    timer_ = this->create_wall_timer(500ms, std::bind(&Simulator::timer_callback, this));
    change_state_timer_ = this->create_wall_timer(5000ms, std::bind(&Simulator::state_callback, this));

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    RCLCPP_INFO(this->get_logger(), "Game simulator node started.");
  }

private:
  void timer_callback()
  {
    // Publish board location
    interfaces::msg::BoardPose board_msg;
    board_msg.point.point.x = 0.35;
    board_msg.point.point.y = 0.35;
    board_msg.point.point.z = 0.02;
    board_msg.anglerad = 0.0;
    board_pub_->publish(board_msg);

    // Publish white pieces
    geometry_msgs::msg::PoseArray white_msg;
    white_msg.header.stamp = this->get_clock()->now();
    white_msg.header.frame_id = "base_link";

    for (size_t i = 0; i < 4; ++i)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.65;
      pose.position.y = 0.15 + 0.1 * i;
      pose.position.z = 0.02;
      pose.orientation.w = 1.0;
      white_msg.poses.push_back(pose);
    }
    white_pub_->publish(white_msg);

    // Publish black pieces
    geometry_msgs::msg::PoseArray black_msg;
    black_msg.header.stamp = this->get_clock()->now();
    black_msg.header.frame_id = "base_link";

    for (size_t i = 0; i < 5; ++i)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.30 + 0.1 * i;
      pose.position.y = 0.05;
      pose.position.z = 0.02;
      pose.orientation.w = 1.0;
      if (i == 3 && change_2_) {
        pose.position.x = 0.46;
        pose.position.y = 0.46;
      }
      if (i == 4 && change_) { // remove black from offboard
        pose.position.x = 0.35;
        pose.position.y = 0.35;
      }
      black_msg.poses.push_back(pose);
    }
    black_pub_->publish(black_msg);

    broadcast_point_tf("board", 0.35, 0.35, 0.02, 0.0);
    broadcast_grid_square_tfs();
  }

  void state_callback() {
    if (change_ == true) change_2_ = true;
    change_ = true;
  }

  void broadcast_point_tf(const std::string &child_frame,
                        double x, double y, double z,
                        double yaw_rad) {
      geometry_msgs::msg::TransformStamped t;

      t.header.stamp = this->get_clock()->now();
      t.header.frame_id = "base_link";
      t.child_frame_id = child_frame;

      t.transform.translation.x = x;
      t.transform.translation.y = y;
      t.transform.translation.z = z;

      // Convert yaw → quaternion
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw_rad);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      tf_broadcaster_->sendTransform(t);
  }

  void broadcast_grid_square_tfs() {
    const double CELL_SIZE = 0.11;
    const std::vector<std::pair<int,int>> GRID_INDEXES = {
        {-1,-1}, {-1,0}, {-1,1},
        {0,-1}, {0,0}, {0,1},
        {1,-1}, {1,0}, {1,1}
    };

    for (size_t i = 0; i < GRID_INDEXES.size(); i++) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "board";
        t.child_frame_id = "board_index_" + std::to_string(i);

        t.transform.translation.x = GRID_INDEXES[i].first * CELL_SIZE;
        t.transform.translation.y = GRID_INDEXES[i].second * CELL_SIZE;
        t.transform.translation.z = 0.0;

        t.transform.rotation.w = 1.0; 

        tf_broadcaster_->sendTransform(t);
    }
  }


  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  rclcpp::Publisher<interfaces::msg::BoardPose>::SharedPtr board_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr white_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr black_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr change_state_timer_;
  bool change_ = false;
  bool change_2_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}
