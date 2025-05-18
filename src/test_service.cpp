#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>

using namespace std;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

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

    RCLCPP_INFO(this->get_logger(), "Service Client Ready");
  }

private:
  void laser_scan_callback(const LaserScan::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Service Request");

    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = *msg;

    if (!direction_service_client_->wait_for_service(1s)) {
      RCLCPP_INFO(this->get_logger(),
                  "service not available, waiting again...");
      return;
    }

    direction_service_client_->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture result) {
          string direction = result.get()->direction;
          RCLCPP_INFO(this->get_logger(), "Service Response - %s",
                      direction.c_str());
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