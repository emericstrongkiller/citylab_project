#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>

using Empty = std_srvs::srv::Empty;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("movement_server") {
    srv_ = create_service<Empty>(
        "/direction_service",
        std::bind(&DirectionService::direction_service_callback, this, _1, _2));
  }

private:
  rclcpp::Service<Empty>::SharedPtr srv_;

  void
  direction_service_callback(const std::shared_ptr<Empty::Request> request,
                             const std::shared_ptr<Empty::Response> response) {}
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}