#include "robot_patrol/srv/get_direction.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <vector>

using namespace std;
using std::placeholders::_1;
using std::placeholders::_2;

class DirectionService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;

  DirectionService() : Node("direction_service_node") {
    direction_service_ = this->create_service<GetDirection>(
        "direction_service",
        bind(&DirectionService::direction_service_callback, this, _1, _2));

    RCLCPP_INFO(this->get_logger(), "Service Server Ready.");
  }

private:
  void direction_service_callback(
      const shared_ptr<GetDirection::Request> request,
      const shared_ptr<GetDirection::Response> response) {
    RCLCPP_INFO(this->get_logger(), "Service Server Requested.");

    vector<float> laser_ranges = request->laser_data.ranges;
    int front_ray = laser_ranges.size() / 2;
    int right_angle_index = front_ray / 2;
    int total_dist_sec =
        (60.0 * M_PI / 180.0) / request->laser_data.angle_increment;

    float total_dist_sec_right = calculate_section(
        right_angle_index, right_angle_index + total_dist_sec, laser_ranges);

    float total_dist_sec_front =
        calculate_section(right_angle_index + total_dist_sec,
                          right_angle_index + total_dist_sec * 2, laser_ranges);

    float total_dist_sec_left =
        calculate_section(right_angle_index + total_dist_sec * 2,
                          front_ray + right_angle_index, laser_ranges);

    float largest_range = max(total_dist_sec_front,
                              max(total_dist_sec_left, total_dist_sec_right));

    if (largest_range == total_dist_sec_front) {
      response->direction = "FRONT";
    } else if (largest_range == total_dist_sec_left) {
      response->direction = "LEFT";
    } else if (largest_range == total_dist_sec_right)
      response->direction = "RIGHT";

    RCLCPP_INFO(this->get_logger(), "Service Server Completed");
  }

  float calculate_section(int start_pos, int end_pos,
                          vector<float> &laser_ranges) {
    float total_dist = 0.0;

    for (int i = start_pos; i < end_pos; i++) {
      if (isinf(laser_ranges[i]))
        continue;

      total_dist += laser_ranges[i];
    }

    return total_dist;
  }

  rclcpp::Service<GetDirection>::SharedPtr direction_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
}