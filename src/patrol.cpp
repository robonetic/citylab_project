#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <math.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <unistd.h>

using namespace std;
using namespace std::chrono_literals;
using placeholders::_1;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {
    subscriber_ = create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 2, bind(&Patrol::laser_scan_callback, this, _1));
    publisher_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = create_wall_timer(100ms, bind(&Patrol::control_loop, this));
  }

private:
  void laser_scan_callback(const sensor_msgs::msg::LaserScan &msg) {
    front_range = (M_PI / 2) / msg.angle_increment;
    front_ray = (0 - msg.angle_min) / msg.angle_increment;
    front_ray_view = (35.0 * M_PI / 180.0) / msg.angle_increment;
  }

  bool is_obstacle_in_front() {
    for (int i = front_ray - front_ray_view; i < front_ray + front_ray_view;
         i++) {
      if (scan_msg.ranges[i] < 0.35) {
        return true;
      }
    }

    return false;
  }

  float calculate_safe_direction() {
    int largest_distance_ray_idx = 0;
    float largest_distance_ray = 0.0;

    for (int i = front_ray - front_range; i <= front_ray + front_range; i++) {
      if (isinf(scan_msg.ranges[i])) {
        continue;
      }

      if (scan_msg.ranges[i] > largest_distance_ray) {
        largest_distance_ray = scan_msg.ranges[i];
        largest_distance_ray_idx = i;
      }
    }

    float direction_ = (scan_msg.angle_min +
                        largest_distance_ray_idx * scan_msg.angle_increment);

    return direction_;
  }

  void control_loop() {
    cmd_vel_msg.linear.x = 0.1;
    cmd_vel_msg.angular.z = 0.0;

    if (is_obstacle_in_front()) {
      cmd_vel_msg.angular.z = calculate_safe_direction() / 2;
    }

    publisher_->publish(cmd_vel_msg);
  }

  int front_range;
  int front_ray;
  int front_ray_view;

  geometry_msgs::msg::Twist cmd_vel_msg;
  sensor_msgs::msg::LaserScan scan_msg;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(make_shared<Patrol>());
  rclcpp::shutdown();
}
