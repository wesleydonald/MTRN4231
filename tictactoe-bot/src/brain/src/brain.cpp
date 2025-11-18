/*
BRAIN NODE -> Control the ur5 to play a game of tictactoe

SUBSCRIBES TO  -> /detected/chessboard
               -> /detected/white/pieces
               -> /detected/black/pieces

IS A CLIENT TO -> gripper node
               -> arm node

ASSUMPTIONS    -> Human plays first with X's
               -> The /detected topics publish in the robot base frame

The idea is we are in the PLAYER_TURN state until we see a piece has moved
onto the board. Then we calculate the best move. We then enter the ROBOT_TURN state
where we send a service request to the arm to move to a certain x, y 
location. When we recieve the completed response from the arm node we
send a service request to the gripper node to close the gripper. Then 
a similar thing for lifting the piece and placing it on the board. 
Lastly we return to home_pose.  
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>
#include <set>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "interfaces/srv/move_arm.hpp"
#include "interfaces/srv/close_gripper.hpp"

#include "tictactoe.cpp" 

using namespace std::chrono_literals;

constexpr bool using_gripper = false; // IMPORTANT REMEMBER TO CHANGE WHEN USING GRIPPER

class Brain : public rclcpp::Node {
public:
  Brain() : Node("brain_node"), game_state_(PLAYER_TURN) {
    game_play_ = std::make_unique<TicTacToe>();

    // Subscriptions for board and pieces
    board_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>("/detected/board", 10, std::bind(&Brain::boardCallback, this, std::placeholders::_1));
    white_pieces_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/detected/pieces/white", 10, std::bind(&Brain::whitePiecesCallback, this, std::placeholders::_1));
    black_pieces_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("/detected/pieces/black", 10, std::bind(&Brain::blackPiecesCallback, this, std::placeholders::_1));

    // Clients
    arm_client_ = this->create_client<interfaces::srv::MoveArm>("arm_service");
    while (!arm_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_INFO(get_logger(), "Arm service not available, waiting...");
    }
    gripper_client_ = this->create_client<interfaces::srv::CloseGripper>("gripper_service");
    if (using_gripper) {
      while (!gripper_client_->wait_for_service(std::chrono::seconds(1))) {
          RCLCPP_INFO(get_logger(), "Gripper service not available, waiting...");
      }
    }
    RCLCPP_INFO(this->get_logger(), "Brain node started and waiting for game input...");

    home_pose_.position.x = 0.458;
    home_pose_.position.y = 0.133;
    home_pose_.position.z = 0.552;
    home_pose_.orientation.x = 1.0;
    home_pose_.orientation.y = 0.0;
    home_pose_.orientation.z = 0.0;
    home_pose_.orientation.w = 0.0;

    sendArmRequest(home_pose_, true);
  }

private:
  void boardCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    board_origin_ = *msg;
    RCLCPP_INFO_ONCE(get_logger(), "Board center set to (%.3f, %.3f, %.3f)", msg->point.x, msg->point.y, msg->point.z);
  }

  void blackPiecesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    std::vector<int> current_on_board = getOnBoardIndices(msg->poses);

    // If it's player's turn, detect new placement
    if (game_state_ == PLAYER_TURN &&
        current_on_board.size() > last_black_on_board_.size()) {

        std::set<int> prev(last_black_on_board_.begin(), last_black_on_board_.end());
        for (int idx : current_on_board) {
            if (prev.find(idx) == prev.end()) {
                RCLCPP_INFO(this->get_logger(), "Detected new black piece on cell %d", idx);
                onHumanMove(idx);
                break;
            }
        }
    }

    last_black_on_board_ = std::move(current_on_board);
  }

  void whitePiecesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    last_white_on_board_ = getOnBoardIndices(msg->poses);

    last_white_off_board_.clear();
    for (size_t i = 0; i < msg->poses.size(); ++i) {
        std::vector<int> idx_vec = getOnBoardIndices({msg->poses[i]});
        if (idx_vec.empty()) {
            last_white_off_board_.push_back(msg->poses[i]);
        }
    }

    //RCLCPP_INFO(get_logger(), "White pieces on board: %zu, off board: %zu",
    //           last_white_on_board_.size(), last_white_off_board_.size());
  }

  // Movement logic
  void onHumanMove(int cell) {
    if (game_play_->makeMove(cell, TicTacToe::BLACK)) {
      game_state_ = ROBOT_TURN;

      int computerMove = game_play_->findBestMove();
      RCLCPP_INFO(get_logger(), "Computer selected move %d", computerMove);

      moveRobotToCell(computerMove);
      game_play_->makeMove(computerMove, TicTacToe::WHITE);
    }
  }

  // Finds which pieces are on the board
  std::vector<int> getOnBoardIndices(const std::vector<geometry_msgs::msg::Pose> &poses) {
    std::vector<int> indices;
    if (board_origin_.header.frame_id.empty()) return indices;

    const double cell_size = TicTacToe::CELL_SIZE;
    const double half_span = 1.6 * cell_size;

    double ox = board_origin_.point.x;
    double oy = board_origin_.point.y;

    for (const auto &p : poses) {
        double dx = p.position.x - ox;
        double dy = p.position.y - oy;

        if (std::fabs(dx) > half_span || std::fabs(dy) > half_span) continue;

        // This logic might be a bit cooked
        int col = static_cast<int>(std::round(dx / cell_size));
        int row = static_cast<int>(std::round(dy / cell_size));

        col = std::max(-1, std::min(1, col));
        row = std::max(-1, std::min(1, row));

        int idx = (row + 1) * 3 + (col + 1);
        if (idx >= 0 && idx < 9) indices.push_back(idx);
    }

    std::sort(indices.begin(), indices.end());
    indices.erase(std::unique(indices.begin(), indices.end()), indices.end());
    return indices;
  }

  // Picking up the piece
  void moveRobotToCell(int cell_index) {
    // Pick first available off-board piece
    target_piece_ = last_white_off_board_.front();
    last_white_off_board_.erase(last_white_off_board_.begin());

    // Compute target cell pose from cell_index
    double cell_x = (cell_index % 3 - 1) * TicTacToe::CELL_SIZE;
    double cell_y = (cell_index / 3 - 1) * TicTacToe::CELL_SIZE;
    target_cell_ = target_piece_;
    target_cell_.position.x = board_origin_.point.x - cell_x;
    target_cell_.position.y = board_origin_.point.y - cell_y;

    current_action_ = MOVE_TO_PICK;
    sendArmRequest(target_piece_, false);
  } 

  void sendArmRequest(const geometry_msgs::msg::Pose &pose, bool move_home) {
    auto req = std::make_shared<interfaces::srv::MoveArm::Request>();
    req->target_pose = pose;
    req->move_home = move_home;
    arm_client_->async_send_request(req, std::bind(&Brain::handleArmResponse, this, std::placeholders::_1));
  }

  void handleArmResponse(rclcpp::Client<interfaces::srv::MoveArm>::SharedFuture future) {
    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(get_logger(), "Arm move failed: %s", response->message.c_str());
        current_action_ = IDLE;
        return;
    }
    RCLCPP_INFO(get_logger(), "Arm move succeeded: %s", response->message.c_str());

    switch (current_action_) {
        case MOVE_TO_PICK:
            RCLCPP_INFO(get_logger(), "Reached piece, closing gripper");
            current_action_ = CLOSE_GRIPPER;
            if (using_gripper) {
                sendGripperRequest(true);
            }
            else {
                current_action_ = MOVE_TO_PLACE;
                sendArmRequest(target_cell_, false);
            }
            break;

        case MOVE_TO_PLACE:
            RCLCPP_INFO(get_logger(), "Reached target cell, opening gripper");
            current_action_ = OPEN_GRIPPER;
            if (using_gripper) {
                sendGripperRequest(false);
            } else {
                current_action_ = MOVE_TO_HOME;
                sendArmRequest(home_pose_, true);
                game_state_ = PLAYER_TURN;
            }
            break;

        case MOVE_TO_HOME:
            RCLCPP_INFO(get_logger(), "Placed piece and moved home");
            current_action_ = IDLE;
            break;

        default:
            RCLCPP_WARN(get_logger(), "Unexpected action in handleArmResponse");
            current_action_ = IDLE;
            break;
    }
  }

  void sendGripperRequest(bool close) {
    auto req = std::make_shared<interfaces::srv::CloseGripper::Request>();
    req->command = close ? "close" : "open";

    gripper_client_->async_send_request(req, std::bind(&Brain::handleGripperResponse, this, std::placeholders::_1));
  }

  void handleGripperResponse(rclcpp::Client<interfaces::srv::CloseGripper>::SharedFuture future) {
    auto response = future.get();
    if (!response->success) {
        RCLCPP_ERROR(get_logger(), "Gripper action failed: %s", response->message.c_str());
        current_action_ = IDLE;
        return;
    }

    if (current_action_ == CLOSE_GRIPPER) {
        RCLCPP_INFO(get_logger(), "Gripper closed, moving to target cell");
        current_action_ = MOVE_TO_PLACE;
        sendArmRequest(target_cell_, false);
    } else {
        RCLCPP_INFO(get_logger(), "Gripper opened, move complete");
        current_action_ = MOVE_TO_HOME;
        sendArmRequest(home_pose_, true);
        game_state_ = PLAYER_TURN;
    }
  }

  enum State { PLAYER_TURN, ROBOT_TURN, FINISHED };
  enum RobotAction { IDLE, MOVE_TO_PICK, CLOSE_GRIPPER, MOVE_TO_PLACE, OPEN_GRIPPER, MOVE_TO_HOME };

  geometry_msgs::msg::Pose home_pose_;

  std::unique_ptr<TicTacToe> game_play_;
  enum State game_state_;
  
  RobotAction current_action_ = IDLE;
  geometry_msgs::msg::Pose target_piece_;
  geometry_msgs::msg::Pose target_cell_;

  std::vector<int> last_black_on_board_;
  std::vector<int> last_white_on_board_;
  std::vector<geometry_msgs::msg::Pose> last_white_off_board_;
  geometry_msgs::msg::PointStamped board_origin_;

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr board_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr white_pieces_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr black_pieces_sub_;

  rclcpp::Client<interfaces::srv::MoveArm>::SharedPtr arm_client_;
  rclcpp::Client<interfaces::srv::CloseGripper>::SharedPtr gripper_client_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}
