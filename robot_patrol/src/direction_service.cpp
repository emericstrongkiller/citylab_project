#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"

#include <memory>

using GetDirection = custom_interfaces::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;
using namespace std;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("movement_server") {
    srv_ = create_service<GetDirection>(
        "/direction_service",
        std::bind(&DirectionService::direction_service_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  // laser part
  float laser_ranges[720]; // current laser scan [0->2pi]rad
  float total_dists_sec[3];

  void direction_service_callback(
      const std::shared_ptr<GetDirection::Request> request,
      const std::shared_ptr<GetDirection::Response> response) {
    // treats laser data and answers forward,right or left
    // 165->275 : total_dist_sec_right
    // 275->385 : total_dist_sec_front
    // 385->495 : total_dist_sec_left
    for (int i = 0; i < 720; i++) {
      laser_ranges[i] = request->laser_data.ranges[i];
    }

    for (int i = 165; i < 275; i++) {
      total_dists_sec[0] += laser_ranges[i];
      total_dists_sec[1] += laser_ranges[i + 110];
      total_dists_sec[2] += laser_ranges[i + 110 * 2];
    }

    switch (distance(total_dists_sec,
                     max_element(total_dists_sec, total_dists_sec + 2))) {
    case 0:
      response->direction = "right";
      break;
    case 1:
      response->direction = "forward";
      break;
    case 2:
      response->direction = "left";
      break;
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}