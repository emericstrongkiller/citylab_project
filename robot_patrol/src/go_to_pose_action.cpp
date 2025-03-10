#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp_action/server.hpp"

#include "/home/simulations/ros2_sims_ws/install/geometry_msgs/include/geometry_msgs/geometry_msgs/msg/pose2_d.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
//#include "geometry_msgs/msg/pose2d.hpp" system would find it if placed here,
// had to do ros2 pkg prefix geometry_msgs to find the path to it then include
// the full path
#include <nav_msgs/msg/odometry.hpp>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class GoToPose : public rclcpp::Node {
public:
  using GoToPose_interface = robot_patrol::action::GoToPose;
  using GoalHandleGoToPose =
      rclcpp_action::ServerGoalHandle<GoToPose_interface>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_node", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<GoToPose_interface>(
        this, "/go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
        std::bind(&GoToPose::handle_cancel, this, _1),
        std::bind(&GoToPose::handle_accepted, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");

    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    subscriber = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GoToPose::odom_callback, this, _1));
  }

private:
  rclcpp_action::Server<GoToPose_interface>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber;

  geometry_msgs::msg::Pose2D desired_pos_;
  geometry_msgs::msg::Pose2D current_pos_;
  float move_vector[3];

  double pos_tolerance = 0.05;
  double angle_tolerance = 0.05;
  double linear_speed = 0.2;  // m/s
  double angular_speed = 0.4; // rad/s

  bool reached_pos_once = false;

  rclcpp::Time last_feedback_time =
      this->now(); // Initialize last feedback time
  rclcpp::Time current_time = this->now();

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose_interface::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    if (goal->goal_pos.x < 1000 && goal->goal_pos.y < 1000 &&
        goal->goal_pos.theta < 1000) {
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Rejected goal: x=%f, y=%f, theta=%f is out of bounds",
                  goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
      return rclcpp_action::GoalResponse::REJECT;
    }
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose_interface::Feedback>();
    feedback->current_pos = current_pos_;
    auto result = std::make_shared<GoToPose_interface::Result>();
    auto move = geometry_msgs::msg::Twist();

    // refresh time
    rclcpp::Time current_time;
    rclcpp::Rate loop_rate(20);

    while (rclcpp::ok()) {
      // reset time to trigger feedback each seconds
      current_time = this->now();

      // compute difference between desired_pos and current_pos
      desired_pos_ = goal->goal_pos;
      desired_pos_.theta = fmod(desired_pos_.theta, 360.0);
      if (desired_pos_.theta < 0) {
        desired_pos_.theta += 360.0;
      }
      // Convert to radians
      desired_pos_.theta = desired_pos_.theta * M_PI / 180.0;

      move_vector[0] = current_pos_.x - desired_pos_.x;
      move_vector[1] = current_pos_.y - desired_pos_.y;
      move_vector[2] = current_pos_.theta - desired_pos_.theta;

      // compute the necessary angle to turn toward the goal
      double angle_to_goal = std::atan2(desired_pos_.y - current_pos_.y,
                                        desired_pos_.x - current_pos_.x);
      double delta_angle = angle_to_goal - current_pos_.theta;

      // Normalize the angle to be within -PI to PI
      while (delta_angle > M_PI)
        delta_angle -= 2.0 * M_PI;
      while (delta_angle < -M_PI)
        delta_angle += 2.0 * M_PI;

      // Check for goal preemption/cancellation
      if (goal_handle->is_canceling()) {
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        move.linear.x = 0.0;
        move.angular.z = 0.0;
        publisher_->publish(move);
        return;
      }

      if ((std::abs(delta_angle) > 0.1 ||
           std::abs(move_vector[0]) > pos_tolerance ||
           std::abs(move_vector[1]) > pos_tolerance) &&
          reached_pos_once == false) {
        move.angular.z = delta_angle * 3;
        move.linear.x = 0.2;
        publisher_->publish(move);
      } else if (std::abs(delta_angle) < 0.1 &&
                 std::abs(move_vector[0]) > pos_tolerance &&
                 std::abs(move_vector[1]) > pos_tolerance &&
                 reached_pos_once == false) {
        move.angular.z = 0;
        move.linear.x = 0.2 * (move_vector[1] + move_vector[0]);
        publisher_->publish(move);
      } else if (std::abs(move_vector[0]) < pos_tolerance &&
                 std::abs(move_vector[1]) < pos_tolerance &&
                 std::abs(move_vector[2]) > angle_tolerance) {
        move.angular.z = -move_vector[2] * 2;
        move.linear.x = 0.0;
        publisher_->publish(move);
        move_vector[2] = current_pos_.theta - desired_pos_.theta;
        reached_pos_once = true;
      } else {
        loop_rate.sleep(); // added this because the robot continued moving if
        move.angular.z = 0.0;
        move.linear.x = 0.0;
        loop_rate.sleep();
        publisher_->publish(move);
        // loop_rate.sleep();
        result->status = true;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        reached_pos_once = false;
        return; // then exit the function when u got the goal !
      }

      // Publish feedback (current position)
      if ((current_time - last_feedback_time).seconds() >= 1.0) {
        feedback->current_pos.x = current_pos_.x;
        feedback->current_pos.y = current_pos_.y;
        feedback->current_pos.theta = current_pos_.theta;
        goal_handle->publish_feedback(feedback);
        last_feedback_time = this->now();
      }

      loop_rate.sleep();
    }
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;

    // quaternion conversion into an angle
    tf2::Quaternion q(
        msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pos_.theta = yaw;
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