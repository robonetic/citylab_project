#include "rclcpp/executors.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/utilities.hpp"
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std;

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("patrol_node") {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(make_shared<Patrol>());
  rclcpp::shutdown();
}
