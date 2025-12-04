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
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "interfaces/msg/board_pose.hpp"
#include "interfaces/srv/move_arm.hpp"
#include "interfaces/srv/close_gripper.hpp"

// For GUI communication
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "tictactoe.cpp" 

using namespace std::chrono_literals;

constexpr bool using_gripper = true; // IMPORTANT REMEMBER TO CHANGE WHEN USING GRIPPER

class Brain : public rclcpp::Node {
public:
  Brain() : Node("brain_node"), game_state_(PLAYER_TURN), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
    game_play_ = std::make_unique<TicTacToe>();

    // Subscriptions for board and pieces
    board_sub_ = this->create_subscription<interfaces::msg::BoardPose>("/detected/board", 10, std::bind(&Brain::boardCallback, this, std::placeholders::_1));
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
      sendGripperRequest(false);
    }

    // --- GUI Publishers / Subscribers
    arm_stop_client_ = this->create_client<std_srvs::srv::Trigger>("arm_stop_service");
    if (!arm_stop_client_->wait_for_service(1s)) {
        RCLCPP_WARN(get_logger(), "Arm STOP service (arm_stop_service) not available. E-Stop will be software-only.");
    }

    status_pub_ = this->create_publisher<std_msgs::msg::String>("/game/status", 10);
    wins_pub_ = this->create_publisher<std_msgs::msg::Int32>("/game/metrics/wins", 10);
    losses_pub_ = this->create_publisher<std_msgs::msg::Int32>("/game/metrics/losses", 10);
    ties_pub_ = this->create_publisher<std_msgs::msg::Int32>("/game/metrics/ties", 10);

    control_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/game/control", 10,
        std::bind(&Brain::control_callback, this, std::placeholders::_1));

    estop_service_ = this->create_service<std_srvs::srv::Trigger>(
        "/ur5e/trigger_estop",
        std::bind(&Brain::estop_callback, this, std::placeholders::_1, std::placeholders::_2));

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
  void boardCallback(const interfaces::msg::BoardPose::SharedPtr msg) {
    if (board_set_) return;
    board_origin_ = msg->point;
    board_orientation_ = msg->anglerad;
    board_set_ = true;
    RCLCPP_INFO_ONCE(get_logger(), "Board center set to (%.3f, %.3f, %.3f)", board_origin_.point.x, board_origin_.point.y, board_orientation_);
  }

  void blackPiecesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (game_state_ != PLAYER_TURN) return;
    std::vector<int> current_on_board = getOnBoardIndices(msg->poses);

    // If it's player's turn, detect new placement
    if (game_state_ == PLAYER_TURN && current_on_board.size() > last_black_on_board_.size()) {
        std::set<int> prev(last_black_on_board_.begin(), last_black_on_board_.end());
        for (int idx : current_on_board) {
            if (prev.find(idx) == prev.end() && game_play_->makeMove(idx, TicTacToe::BLACK)) {
                RCLCPP_INFO(this->get_logger(), "Detected new black piece on cell %d", idx);
                onHumanMove();
                break;
            }
        }
    }

    last_black_on_board_ = std::move(current_on_board);
  }

  void whitePiecesCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
    if (game_state_ != PLAYER_TURN) return;
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
  void onHumanMove() {
    game_state_ = ROBOT_TURN;

    int computerMove = game_play_->findBestMove();
    RCLCPP_INFO(get_logger(), "Computer selected move %d", computerMove);

    if (computerMove != -1) {
        moveRobotToCell(computerMove);
        game_play_->makeMove(computerMove, TicTacToe::WHITE);
    } else {
        RCLCPP_INFO(get_logger(), "GAME FINISHED");
    }
  }

  // Finds which pieces are on the board
  std::vector<int> getOnBoardIndices(const std::vector<geometry_msgs::msg::Pose> &poses) {
    std::vector<int> indices;
    constexpr double MAX_DIST = 0.05;

    // Pre-fetch square positions in base_link frame
    std::array<geometry_msgs::msg::Point, 9> square_positions;
    for (int i = 0; i < 9; ++i) {
        std::string frame = "board_index_" + std::to_string(i);
        try {
            auto tf = tf_buffer_.lookupTransform("base_link", frame, tf2::TimePointZero);
            square_positions[i].x = tf.transform.translation.x;
            square_positions[i].y = tf.transform.translation.y;
            square_positions[i].z = tf.transform.translation.z;
        } catch (...) { }
    }

    for (const auto &p : poses) {
        // Find nearest square
        double best_dist = 1e9;
        int best_idx = -1;
        //RCLCPP_INFO(get_logger(), "Piece at (%lf %lf)", p.position.x, p.position.y);

        for (int i = 0; i < 9; ++i) {
            const auto &sq = square_positions[i];
            if (std::isnan(sq.x)) continue;

            double dx = p.position.x - sq.x;
            double dy = p.position.y - sq.y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist < best_dist) {
                best_dist = dist;
                best_idx = i;
            }
        }

        // Only accept if within 5 cm
        if (best_idx != -1 && best_dist <= MAX_DIST)
            indices.push_back(best_idx);
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

    std::string frame = "board_index_" + std::to_string(cell_index);
    double targetX = 0.0;
    double targetY = 0.0;
    try {
        auto tf = tf_buffer_.lookupTransform("base_link", frame, tf2::TimePointZero);
        targetX = tf.transform.translation.x;
        targetY = tf.transform.translation.y;
    } catch (...) {
        RCLCPP_ERROR(get_logger(), "Failed to lookup TF for %s", frame.c_str());
        return; // ERROR
    }

    target_cell_ = target_piece_;
    target_cell_.position.x = targetX;
    target_cell_.position.y = targetY;
    RCLCPP_INFO(get_logger(), "TARGET CELL AT %lf %lf", target_cell_.position.x, target_cell_.position.y);

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
        //current_action_ = IDLE;
        //return;
    }
    RCLCPP_INFO(get_logger(), "Arm move succeeded: %s", response->message.c_str());

    switch (current_action_) {
        case MOVE_TO_PICK:
            RCLCPP_INFO(get_logger(), "Reached piece, closing gripper");
            current_action_ = CLOSE_GRIPPER;
            if (using_gripper) {
                sendGripperRequest(true);
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
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
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            } else {
                current_action_ = MOVE_TO_HOME;
                sendArmRequest(home_pose_, true);
            }
            break;

        case MOVE_TO_HOME:
            RCLCPP_INFO(get_logger(), "Placed piece and moved home");
            game_state_ = PLAYER_TURN;
            current_action_ = IDLE;
            break;

        case IDLE:
            RCLCPP_INFO(get_logger(), "Arm Idling (likely from Start/End/E-Stop or already at pose).");
            // No further action is needed. The state is already IDLE.
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

  // --- GUI Communication Functions ---

  // This function sends all current counts to the GUI
  void publish_all_metrics()
  {
      wins_pub_->publish(std_msgs::msg::Int32().set__data(session_wins_));
      losses_pub_->publish(std_msgs::msg::Int32().set__data(session_losses_));
      ties_pub_->publish(std_msgs::msg::Int32().set__data(session_ties_));
  }

  // This function runs when "Start" or "End" is clicked
  // This function runs when "Start" or "End" is clicked
  void control_callback(const std_msgs::msg::String & msg)
  {
      std::string command = msg.data;
      RCLCPP_INFO(get_logger(), "Received command from GUI: '%s'", command.c_str());

      if (command == "start") {
          // Reset the game board
          game_play_->resetBoard();
          game_state_ = PLAYER_TURN;
          
          // Let the user know the game is starting
          status_pub_->publish(std_msgs::msg::String().set__data("New game started... It's your turn!"));
          
          // --- MODIFIED: Tell robot to move to home pose ---
          RCLCPP_INFO(get_logger(), "Moving arm to home position to start game...");
          // Set action to IDLE to avoid conflicts in the arm response handler
          current_action_ = IDLE; 
          sendArmRequest(home_pose_, true);
          // --------------------------------------------------
          
      } else if (command == "end") {
          
          // --- MODIFIED: Tell robot to move to home pose ---
          RCLCPP_INFO(get_logger(), "Ending session, moving arm to home position.");
          // Set state to finished and action to IDLE to stop any current/future moves
          game_state_ = FINISHED;
          current_action_ = IDLE;
          sendArmRequest(home_pose_, true);
          // --------------------------------------------------

          // Reset all session counters
          session_wins_ = 0;
          session_losses_ = 0;
          session_ties_ = 0;
          
          // Publish the "0" counts to the GUI to reset it
          publish_all_metrics();
          status_pub_->publish(std_msgs::msg::String().set__data("Session ended. Ready for a new game?"));
      }
  }
  
  // This function runs when "E-Stop" is clicked
  void estop_callback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
      // Use the request parameter to avoid compiler warning
      (void)request; 

      RCLCPP_ERROR(get_logger(), "!!! E-STOP TRIGGERED BY GUI !!!");
      
      // 1. Immediately stop the internal state machine
      RCLCPP_INFO(get_logger(), "Halting brain node state machine.");
      game_state_ = FINISHED;
      current_action_ = IDLE;

      // 2. Call the hardware/arm stop service (Fire and Forget)
      if (arm_stop_client_ && arm_stop_client_->service_is_ready()) {
          RCLCPP_INFO(get_logger(), "Calling hardware arm stop service...");
          auto stop_req = std::make_shared<std_srvs::srv::Trigger::Request>();
          
          // --- MODIFICATION ---
          // Just send the request. Do NOT wait for the future.
          // The main 'spin' loop will handle the future's completion.
          // We return our *own* service response immediately.
          arm_stop_client_->async_send_request(stop_req);
          
          response->success = true;
          response->message = "E-Stop confirmed. Arm stop request SENT.";
          // --------------------

      } else {
          RCLCPP_WARN(get_logger(), "Arm stop service not available. E-Stop is software-only.");
          response->success = true;
          response->message = "E-Stop confirmed (software only).";
      }
  }
  // NEW: Function to handle game over logic
  void handle_game_over(int winner)
  {
    if (winner == TicTacToe::BLACK) {
        // Human wins
        session_losses_ += 1; // A loss for the robot
        status_pub_->publish(std_msgs::msg::String().set__data("You win! Great job!"));
    } else if (winner == TicTacToe::WHITE) {
        // Robot wins
        session_wins_ += 1;
        status_pub_->publish(std_msgs::msg::String().set__data("Robot wins. Good game!"));
    } else {
        // Tie
        session_ties_ += 1;
        status_pub_->publish(std_msgs::msg::String().set__data("It's a tie! Well played!"));
    }
    
    publish_all_metrics(); // Send updated counts
    game_state_ = FINISHED;

    // You could add logic here to automatically start a new game
    // or wait for the "Start" button to be pressed.
    // For now, it will just wait.
  }

  enum State { PLAYER_TURN, ROBOT_TURN, FINISHED };
  enum RobotAction { IDLE, MOVE_TO_PICK, CLOSE_GRIPPER, MOVE_TO_PLACE, OPEN_GRIPPER, MOVE_TO_HOME };

  geometry_msgs::msg::Pose home_pose_;

  std::unique_ptr<TicTacToe> game_play_;
  enum State game_state_;
  bool board_set_ = false;
  
  RobotAction current_action_ = IDLE;
  geometry_msgs::msg::Pose target_piece_;
  geometry_msgs::msg::Pose target_cell_;

  std::vector<int> last_black_on_board_;
  std::vector<int> last_white_on_board_;
  std::vector<geometry_msgs::msg::Pose> last_white_off_board_;
  geometry_msgs::msg::PointStamped board_origin_;
  double board_orientation_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<interfaces::msg::BoardPose>::SharedPtr board_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr white_pieces_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr black_pieces_sub_;

  rclcpp::Client<interfaces::srv::MoveArm>::SharedPtr arm_client_;
  rclcpp::Client<interfaces::srv::CloseGripper>::SharedPtr gripper_client_;

  // --- GUI Communication Variables ---
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr wins_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr losses_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr ties_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr control_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr estop_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr arm_stop_client_;
  int session_wins_ = 0;
  int session_losses_ = 0;
  int session_ties_ = 0;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Brain>());
  rclcpp::shutdown();
  return 0;
}
