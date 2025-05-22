#include "robot_patrol/srv/get_direction.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <unistd.h>

using namespace std;
using namespace std::chrono_literals;
using placeholders::_1;

class PatrolWithService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Twist = geometry_msgs::msg::Twist;

  PatrolWithService() : Node("patrol_node") {
    cmd_vel_pub_ = create_publisher<Twist>("cmd_vel", 10);
    laser_scan_sub_ = create_subscription<LaserScan>(
        "scan", 2, bind(&PatrolWithService::laser_scan_callback, this, _1));
    direction_service_client_ =
        create_client<GetDirection>("direction_service");
    control_loop_timer_ =
        create_wall_timer(100ms, bind(&PatrolWithService::control_loop, this));

    RCLCPP_INFO(get_logger(), "Initialized, Service Client Ready!");
  }

private:
  void laser_scan_callback(const LaserScan::SharedPtr msg) {
    laser_scan_msg = msg;
    front_ray = laser_scan_msg->ranges.size() / 2;
    front_ray_view = (25.0 * M_PI / 180.0) / msg->angle_increment;
  }

  void control_loop() {
    if (!laser_scan_msg) {
      RCLCPP_ERROR(get_logger(), "Laser Scan Not Initialized");
      return;
    }

    if (is_obstacle_in_front())
      calculate_safe_direction();

    if (direction_ == "RIGHT")
      move(0.1, -0.5);

    if (direction_ == "FRONT")
      move(0.1, 0.0);

    if (direction_ == "LEFT")
      move(0.1, 0.5);
  }

  bool is_obstacle_in_front() {
    for (int i = front_ray - front_ray_view; i <= front_ray + front_ray_view;
         i++) {
      if (laser_scan_msg->ranges[i] < 0.35)
        return true;
    }

    direction_ = "FRONT";

    return false;
  }

  void calculate_safe_direction() {
    RCLCPP_INFO(this->get_logger(), "Service Request");
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *laser_scan_msg;

    if (!direction_service_client_->wait_for_service(1s)) {
      RCLCPP_WARN(get_logger(), "Direction service not available");
      return;
    }

    direction_service_client_->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture result) {
          direction_ = result.get()->direction;
          RCLCPP_INFO(this->get_logger(), "Service Response - %s",
                      direction_.c_str());
        });
  }

  void move(float linear, float angular) {
    cmd_vel_msg.linear.x = linear;
    cmd_vel_msg.angular.z = angular;
    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  string direction_;
  int front_ray = 0;
  int front_range = 0;
  int front_ray_view = 0;

  Twist cmd_vel_msg;
  LaserScan::SharedPtr laser_scan_msg;

  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Client<GetDirection>::SharedPtr direction_service_client_;
  rclcpp::TimerBase::SharedPtr control_loop_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<PatrolWithService>());
  rclcpp::shutdown();
}