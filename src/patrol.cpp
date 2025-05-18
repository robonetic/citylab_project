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

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    laser_scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 2, bind(&Patrol::laser_scan_callback, this, _1));
    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(100ms, bind(&Patrol::control_loop, this));

    RCLCPP_INFO(get_logger(), "Robot initialized - Ready for patrol!");
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    laser_scan_msg = msg;
    front_ray = laser_scan_msg->ranges.size() / 2;
    front_range = front_ray / 2;
    front_ray_view = (25.0 * M_PI / 180.0) / msg->angle_increment;
  }

  float calculate_safe_direction() {
    int largest_distance_ray_idx = 0;
    float largest_distance_ray = 0.0;

    for (int i = front_ray - front_range; i <= front_ray + front_range; i++) {
      if (isinf(laser_scan_msg->ranges[i])) {
        continue;
      }

      if (laser_scan_msg->ranges[i] > largest_distance_ray) {
        largest_distance_ray = laser_scan_msg->ranges[i];
        largest_distance_ray_idx = i;
      }
    }

    float direction_ =
        laser_scan_msg->angle_min +
        largest_distance_ray_idx * laser_scan_msg->angle_increment;

    return direction_;
  }

  bool is_obstacle_in_front() {
    for (int i = front_ray - front_ray_view; i <= front_ray + front_ray_view;
         i++) {
      if (laser_scan_msg->ranges[i] < 0.35) {
        RCLCPP_INFO(get_logger(),
                    "Obstacle detected - Calucating greatest distance...");
        return true;
      }
    }

    return false;
  }

  void control_loop() {
    if (!laser_scan_msg) {
      RCLCPP_ERROR(get_logger(), "Laser scan message has not been received");
      return;
    }

    cmd_vel_msg.linear.x = 0.1;
    cmd_vel_msg.angular.z = 0.0;

    if (is_obstacle_in_front()) {
      cmd_vel_msg.angular.z = calculate_safe_direction() / 2;
      RCLCPP_INFO(get_logger(), "Rotating on angular z - %f",
                  cmd_vel_msg.angular.z);
    }

    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  int front_ray = 0;
  int front_range = 0;
  int front_ray_view = 0;

  geometry_msgs::msg::Twist cmd_vel_msg;
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Patrol>());
  rclcpp::shutdown();
}