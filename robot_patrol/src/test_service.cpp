#include "custom_interfaces/srv/get_direction.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class LaserSub : public rclcpp::Node {
public:
  LaserSub() : Node("laser_sub") {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&LaserSub::laser_callback, this, _1));
    client = this->create_client<custom_interfaces::srv::GetDirection>(
        "/direction_service");
    RCLCPP_WARN(this->get_logger(), "direction Client Ready");
  }

private:
  void laser_callback(const sensor_msgs::msg::LaserScan msg) {
    request = std::make_shared<custom_interfaces::srv::GetDirection::Request>();
    request->laser_data = msg;
    send_async_request();
  }

  void send_async_request() {
    if (!client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_WARN(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto result_future = client->async_send_request(
        request,
        std::bind(&LaserSub::response_callback, this, std::placeholders::_1));
    RCLCPP_WARN(this->get_logger(), "direction requested to direction server");

    // Now check for the response after a timeout of 1 second
    auto status = result_future.wait_for(1s);

    if (status != std::future_status::ready) {
      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }

  void response_callback(
      rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedFuture
          future) {
    // Get response value
    auto response = future.get();
    RCLCPP_INFO(this->get_logger(),
                "Direction Server responded\nDirection chosen: %s",
                response->direction.c_str());
    rclcpp::shutdown();
  }

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<custom_interfaces::srv::GetDirection>::SharedPtr client;

  sensor_msgs::msg::LaserScan laser_data;
  custom_interfaces::srv::GetDirection::Request::SharedPtr request;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserSub>();
  rclcpp::spin(node);
  return 0;
}

/*
Inside the robot_patrol package create a new C++ file named test_service.cpp.

Inside this file, create a simple node that does the following:

It subscribes to the laser data.
In the callback, it calls the /direction_service service with the proper data.
It prints the response returned by the service.
Create a launch file named start_test_service.launch.py that launches the
test_service.cpp program.

Put the simulated robot in a proper location for your test. You can do that
using the teleop_twist_keyboard node (see Notes below).

Launch the service server node using the start_direction_service.launch.py file.

Launch the tester node using the start_test_service.launch.py file.
*/