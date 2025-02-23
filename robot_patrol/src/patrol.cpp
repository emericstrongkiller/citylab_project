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
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;

  rclcpp::CallbackGroup::SharedPtr laser_callback_group_;
  rclcpp::CallbackGroup::SharedPtr cmd_vel_callback_group;

  rclcpp::TimerBase::SharedPtr publish_timer_;

  sensor_msgs::msg::LaserScan laser_data;
  geometry_msgs::msg::Twist move_data;

  float direction_;
  float max_laser_dist = 0;
  float max_dist_index = 0;
  bool too_close = false;

  void laser_callback(const sensor_msgs::msg::LaserScan msg) {
    for (int i = 270; i < 390; i++) {
      if (msg.ranges[i] < 0.35) {
        too_close = true;
        break;
      } else {
        too_close = false;
      }
    }

    // update max laser distance
    for (int i = 165; i < 495; i++) {
      if (!std::isinf(msg.ranges[i]) && msg.ranges[i] > max_laser_dist) {
        max_laser_dist = msg.ranges[i];
        max_dist_index = i;
      }
    }

    // calculate angle to attain
    direction_ = -1 * (msg.angle_increment * (330 - max_dist_index));
  }

  void cmd_vel_callback() {
    if (too_close == true) {
      move_data.linear.x = 0.1;
      move_data.angular.z = direction_ / 2;
      cmd_vel_pub->publish(move_data);
    } else {
      move_data.linear.x = 0.1;
      move_data.angular.z = 0.0;
      cmd_vel_pub->publish(move_data);
    }
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