#include "robot_patrol/srv/get_direction.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using GetDirection = robot_patrol::srv::GetDirection;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    direction_service_ = this->create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::direction_service_callback, this, _1, _2));
  }

private:
  void direction_service_callback(
      const std::shared_ptr<GetDirection::Request> request,
      const std::shared_ptr<GetDirection::Response> response) {

    RCLCPP_INFO(
        get_logger(),
        "Direction service called, calculating values in each section.");

    int section = (60.0 * M_PI / 180.0) / request->laser_data.angle_increment;
    int front_ray = request->laser_data.ranges.size() / 2;
    int starting_index = front_ray / 2;

    float total_dist_sec_right = 0.0;
    float total_dist_sec_front = 0.0;
    float total_dist_sec_left = 0.0;

    for (int i = starting_index; i < starting_index + section; i++) {
      if (std::isinf(request->laser_data.ranges[i])) {
        continue;
      }

      total_dist_sec_right += request->laser_data.ranges[i];
    }

    for (int i = starting_index + section; i < starting_index + section * 2;
         i++) {
      if (std::isinf(request->laser_data.ranges[i])) {
        continue;
      }

      total_dist_sec_front += request->laser_data.ranges[i];
    }

    for (int i = starting_index + section * 2; i < starting_index + section * 3;
         i++) {
      if (std::isinf(request->laser_data.ranges[i])) {
        continue;
      }

      total_dist_sec_left += request->laser_data.ranges[i];
    }

    float larget_range = std::max(total_dist_sec_left, total_dist_sec_right);

    if (total_dist_sec_front > 35) {
      RCLCPP_INFO(
          get_logger(),
          "Front distance is greater than 35cm, continue moving forward");
      response->direction = "FRONT";
      return;
    }

    if (larget_range == total_dist_sec_left) {
      response->direction = "LEFT";
      RCLCPP_INFO(get_logger(), "Left is the greatest distance, turn left");
      return;
    }

    if (larget_range == total_dist_sec_right) {
      response->direction = "RIGHT";
      RCLCPP_INFO(get_logger(), "Right is the greatest distance, turn right");
      return;
    }
  }

  rclcpp::Service<GetDirection>::SharedPtr direction_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
}