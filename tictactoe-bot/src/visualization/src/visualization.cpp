/*
VISUALISATION NODE -> Publishes markers to rviz for system visualisation

SUBSCRIBES TO  -> /detected/board (point_stamped message?)
               -> /detected/white/pieces (pose array)
               -> /detected/black/pieces (pose array)

PUBLISHES TO   -> /tictactoe/white_markers (marker array)
               -> /tictactoe/black_markers (marker array)
               -> /tictactoe/board_marker (single marker)
               -> /tictactoe/gripper_marker (single marker)

TODO           -> the open version of the gripper is at an incorrect orientation so need to change that
               -> need to update gripper image depending on game state, may need to make a new subscriber?

Currently this node just publishes the markers and end 
effector to their positions. Will later add a more dynamic visualisation
that can show the piece being picked up with the end effector.
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include "interfaces/msg/board_pose.hpp"

using namespace std::chrono_literals;

class Visualisation : public rclcpp::Node {
public:
  Visualisation() : Node("visualisation") {
    // Subscriptions
    board_sub_ = this->create_subscription<interfaces::msg::BoardPose>("/detected/board", 10, std::bind(&Visualisation::boardCallback, this, std::placeholders::_1));
    white_pieces_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/detected/pieces/white", 10, std::bind(&Visualisation::whitePiecesCallback, this, std::placeholders::_1));
    black_pieces_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/detected/pieces/black", 10, std::bind(&Visualisation::blackPiecesCallback, this, std::placeholders::_1));

    // Publishers
    white_piece_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tictactoe/white_markers", 1);
    black_piece_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("tictactoe/black_markers", 1);
    board_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tictactoe/board_marker", 1);
    gripper_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("tictactoe/gripper_marker", 1);

    gripper_timer_ = this->create_wall_timer(10ms, std::bind(&Visualisation::gripper_callback, this));
    RCLCPP_INFO(this->get_logger(), "Visualisation node started.");
  }

private:
  // Callback will be used to publish the gripper dynamically
  // TODO: Edit this to change based on the game state
  void gripper_callback() {
    gripper_publisher_->publish(create_marker("tool0", "GripperClosed.stl", 0.0, 0.0, 0.05, // GripperOpen is not oriented correctly for some reason
                                                                          0.6, 0.6, 0.6, 
                                                                          0, 0.0));
  }
  // Creates a marker to be published (orienation is always vertically upwards)
  visualization_msgs::msg::Marker create_marker(const std::string &frame_id, const std::string &mesh_file,
                                                double x, double y, double z, float r, float g, float b, int id, double mYaw) {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "basic_shapes";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::MESH_RESOURCE;

    marker.mesh_resource = "package://visualization/meshes/" + mesh_file;  // e.g. "GripperOpen.stl"
    marker.action = visualization_msgs::msg::Marker::ADD;

    tf2::Quaternion q;
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = mYaw;

    q.setRPY(roll, pitch, yaw);

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();

    marker.scale.x = 0.001;
    marker.scale.y = 0.001;
    marker.scale.z = 0.001;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    return marker;
  }

  void boardCallback(const interfaces::msg::BoardPose::SharedPtr msg) {
    board_publisher_->publish(create_marker("base_link", "Board.stl",
                                      msg->point.point.x, msg->point.point.y, msg->point.point.z,
                                      0.0f, 0.0f, 0.0f, 1, msg->anglerad));
  }

  void whitePiecesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 10;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    for (const auto &pose : msg->poses) {
      auto marker = create_marker("base_link", "O_TicTacToe.stl",
                                  pose.position.x, pose.position.y, pose.position.z,
                                  0.95f, 0.95f, 0.95f, id++, 0.0);
      marker.pose.orientation = pose.orientation;
      marker_array.markers.push_back(marker);
    }

    white_piece_publisher_->publish(marker_array);
  }

  void blackPiecesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    visualization_msgs::msg::MarkerArray marker_array;
    int id = 20;

    visualization_msgs::msg::Marker delete_all;
    delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_all);

    for (const auto &pose : msg->poses) {
      auto marker = create_marker("base_link", "X_TicTacToe.stl",
                                  pose.position.x, pose.position.y, pose.position.z,
                                  0.1f, 0.1f, 0.1f, id++, 0.0);
      marker.pose.orientation = pose.orientation;
      marker_array.markers.push_back(marker);
    }

    black_piece_publisher_->publish(marker_array);
  }

  // Members
  rclcpp::TimerBase::SharedPtr gripper_timer_;

  rclcpp::Subscription<interfaces::msg::BoardPose>::SharedPtr board_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr white_pieces_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr black_pieces_sub_;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr white_piece_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr black_piece_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr board_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr gripper_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Visualisation>());
  rclcpp::shutdown();
  return 0;
}
