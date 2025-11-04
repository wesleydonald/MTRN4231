/*
SIMULATOR NODE -> Publishes piece and board locations when away from the lab
                  used for testing other packages.

PUBLISHES TO   -> /detected/chessboard
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
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

class Simulator : public rclcpp::Node {
public:
  Simulator() : Node("simulator") {
    // Publishers
    board_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/detected/chessboard", 10);
    white_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/white/pieces", 10);
    black_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/detected/black/pieces", 10);

    // Timer
    timer_ = this->create_wall_timer(500ms, std::bind(&Simulator::timer_callback, this));
    change_state_timer_ = this->create_wall_timer(10000ms, std::bind(&Simulator::state_callback, this));

    RCLCPP_INFO(this->get_logger(), "Game simulator node started.");
  }

private:
  void timer_callback()
  {
    // Publish board location
    geometry_msgs::msg::PointStamped board_msg;
    board_msg.header.stamp = this->get_clock()->now();
    board_msg.header.frame_id = "base_link";
    board_msg.point.x = 0.35;
    board_msg.point.y = 0.50;
    board_msg.point.z = 0.0;
    board_pub_->publish(board_msg);

    // Publish white pieces
    geometry_msgs::msg::PoseArray white_msg;
    white_msg.header.stamp = this->get_clock()->now();
    white_msg.header.frame_id = "base_link";

    for (size_t i = 0; i < 4; ++i)
    {
      geometry_msgs::msg::Pose pose;
      pose.position.x = 0.65;
      pose.position.y = 0.3 + 0.1 * i;
      pose.position.z = 0.0;
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
      pose.position.y = 0.20;
      pose.position.z = 0.0;
      pose.orientation.w = 1.0;
      if (i == 4 && change_) { // remove black from offboard
        pose.position.x = 0.35;
        pose.position.y = 0.5;
      }
      black_msg.poses.push_back(pose);
    }
    black_pub_->publish(black_msg);
  }

  void state_callback() {
    change_ = true;
  }

  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr board_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr white_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr black_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr change_state_timer_;
  bool change_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simulator>());
  rclcpp::shutdown();
  return 0;
}
