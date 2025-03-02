#include <functional>
#include <memory>
#include <thread>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "t3_action_msg/action/move.hpp"

#include "custom_interfaces/action/go_to_pose.hpp"
#include "geometry_msgs/msg/pose2d.hpp"
#include <nav_msgs/msg/odometry.hpp>

class GoToPose : public rclcpp::Node {
public:
  using Move = t3_action_msg::action::Move;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&Patrol::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<Move>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscriber<nav_msgs::msg::Odometry>::SharedPtr subscriber;

  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const Move::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d",
                goal->secs);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Move::Feedback>();
    auto &message = feedback->feedback;
    message = "Starting movement...";
    auto result = std::make_shared<Move::Result>();
    auto move = geometry_msgs::msg::Twist();
    rclcpp::Rate loop_rate(1);

    for (int i = 0; (i < goal->secs) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Move robot forward and send feedback
      message = "Moving forward...";
      move.linear.x = 0.3;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry msg) {
    current_pos_.x = msg.x;
    current_pos_.y = msg.y;
    current_pos_.theta = msg.theta;
  }
}; // class GoToPose

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPose>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}