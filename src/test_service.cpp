#include "rclcpp/executors.hpp"
#include "rclcpp/future_return_code.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TestDirectionService : public rclcpp::Node {
public:
  using GetDirection = robot_patrol::srv::GetDirection;
  using LaserScan = sensor_msgs::msg::LaserScan;

  TestDirectionService() : Node("test_service_node") {
    direction_service_client_ =
        this->create_client<GetDirection>("direction_service");
    laser_scan_sub_ = this->create_subscription<LaserScan>(
        "scan", 2,
        std::bind(&TestDirectionService::laser_scan_callback, this, _1));
  }

private:
  void laser_scan_callback(const LaserScan::SharedPtr msg) {
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;

    while (!direction_service_client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                  "service not available, waiting again...");
    }

    auto result = direction_service_client_->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture result) {
          RCLCPP_INFO(get_logger(), "Service Complete - Direction: %s",
                      result.get()->direction);
          rclcpp::shutdown();
        });
  }

  rclcpp::Client<GetDirection>::SharedPtr direction_service_client_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_scan_sub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestDirectionService>());
  rclcpp::shutdown();
}