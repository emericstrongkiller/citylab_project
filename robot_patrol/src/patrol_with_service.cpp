#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <unistd.h>

using namespace std::chrono_literals;

class Patrol : public rclcpp::Node {
public:
  Patrol(std::string sub_topic, std::string pub_topic) : Node("patrol_node") {
    // laser subscriber part
    laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = laser_callback_group_;

    client_ = this->create_client<custom_interfaces::srv::GetDirection>(
        "/direction_service");

    laser_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        sub_topic, 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1),
        options1);

    // cmd_vel publisher part
    cmd_vel_callback_group = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    cmd_vel_pub =
        this->create_publisher<geometry_msgs::msg::Twist>(pub_topic, 10);

    publish_timer_ = this->create_wall_timer(
        100ms, std::bind(&Patrol::cmd_vel_callback, this),
        cmd_vel_callback_group);
  }

private:
  custom_interfaces::srv::GetDirection::Request::SharedPtr request;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cmd_vel_callback_group;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  sensor_msgs::msg::LaserScan laser_data;
  geometry_msgs::msg::Twist move_data;

  bool too_close = false;
  float direction_ = 0;
  bool service_request_sent = false;

  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_data.ranges.resize(msg->ranges.size());

    if (too_close && !service_request_sent) {
      service_request_sent = true;

      request =
          std::make_shared<custom_interfaces::srv::GetDirection::Request>();
      request->laser_data = laser_data;

      auto result_future = client_->async_send_request(
          request,
          std::bind(&Patrol::response_callback, this, std::placeholders::_1));
    }

    // Check proximity threshold
    for (size_t i = msg->ranges.size() * 3 / 8; i < msg->ranges.size() * 5 / 8;
         i++) {
      if (laser_data.ranges[i] < 0.35 && !service_request_sent) {
        service_request_sent = true;
        too_close = true;
        request =
            std::make_shared<custom_interfaces::srv::GetDirection::Request>();
        request->laser_data = *msg;

        auto result_future = client_->async_send_request(
            request,
            std::bind(&Patrol::response_callback, this, std::placeholders::_1));
        break;
      } else {
        too_close = false;
      }
    }
  }

  void cmd_vel_callback() {
    if (too_close == true) {
      move_data.linear.x = 0.1;
      move_data.angular.z = direction_;
      // RCLCPP_INFO(this->get_logger(), "direction: %f", direction_);
      cmd_vel_pub->publish(move_data);
    } else {
      move_data.linear.x = 0.1;
      move_data.angular.z = 0.0;
      cmd_vel_pub->publish(move_data);
    }
  }

  void response_callback(
      rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture
          future) {
    // Get response value
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(), "Direction chosen: %s",
                response->direction.c_str());
    if (response->direction == "right") {
      direction_ = -0.5;
    } else if (response->direction == "left") {
      direction_ = 0.5;
    } else {
      direction_ = 0;
    }
    service_request_sent = false;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  // node instantiation
  std::shared_ptr<Patrol> patrol_node =
      std::make_shared<Patrol>("/scan", "/cmd_vel");

  RCLCPP_INFO(patrol_node->get_logger(), "Node initialized !");

  // Multithreaded Executor creation
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(patrol_node);
  executor.spin();

  // Shut down and exit.
  rclcpp::shutdown();
  return 0;
}