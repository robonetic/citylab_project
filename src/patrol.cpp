#include "rclcpp/logging.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <iostream>
#include <iterator>
#include <math.h>
#include <memory>
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
    scan_msg = msg;
  }

  void control_loop() {
    int front_ray = (0 - scan_msg.angle_min) / scan_msg.angle_increment;
    int front_ray_view = (20.0 * M_PI / 180.0) / scan_msg.angle_increment;
    int front_range = (M_PI / 2) / scan_msg.angle_increment;
    int largest_distance_ray_idx = 0;
    float largest_distance_ray = 0.0;
    bool obstacle_detected = false;

    cmd_vel_msg.linear.x = 0.1;
    cmd_vel_msg.angular.z = 0.0;

    for (int i = front_ray - front_ray_view; i < front_ray + front_ray_view;
         i++) {
      if (scan_msg.ranges[i] < 0.35) {

        obstacle_detected = true;
      }
    }

    if (obstacle_detected) {
      for (int ray = front_ray - front_range; ray <= front_ray + front_range;
           ray++) {
        if (isinf(scan_msg.ranges[ray])) {
          continue;
        }

        if (scan_msg.ranges[ray] > largest_distance_ray) {
          largest_distance_ray = scan_msg.ranges[ray];
          largest_distance_ray_idx = ray;
        }
      }
      float direction_ = (scan_msg.angle_min +
                          largest_distance_ray_idx * scan_msg.angle_increment);

      cmd_vel_msg.angular.z = direction_ / 2;
    }

    publisher_->publish(cmd_vel_msg);
  }

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
