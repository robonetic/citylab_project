#include "rclcpp/subscription.hpp"
#include "robot_patrol/action/go_to_pose.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <iterator>
#include <math.h>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <thread>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class GoToPose : public rclcpp::Node {
public:
  using Pose2D = geometry_msgs::msg::Pose2D;
  using Twist = geometry_msgs::msg::Twist;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Quaternion = geometry_msgs::msg::Quaternion;
  using Odometry = nav_msgs::msg::Odometry;
  using GoalHandleGoToPose =
      rclcpp_action::ServerGoalHandle<robot_patrol::action::GoToPose>;

  explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_node", options) {
    odom_sub_ = this->create_subscription<Odometry>(
        "odom", 2, std::bind(&GoToPose::odom_sub_callback, this, _1));
    laser_scan_sub_ = this->create_subscription<LaserScan>(
        "scan", 2, std::bind(&GoToPose::laser_scan_callback, this, _1));
    cmd_vel_pub_ = this->create_publisher<Twist>("cmd_vel", 2);
    go_to_pose_service_ =
        rclcpp_action::create_server<robot_patrol::action::GoToPose>(
            this, "go_to_pose", std::bind(&GoToPose::handle_goal, this, _1, _2),
            std::bind(&GoToPose::handle_cancel, this, _1),
            std::bind(&GoToPose::handle_accepted, this, _1));
    control_loop_timer_ = this->create_wall_timer(
        100ms, std::bind(&GoToPose::control_loop, this));
  }

private:
  struct EulerAngles {
    double roll, pitch, yaw;
  };

  void odom_sub_callback(const Odometry::SharedPtr msg) {
    Quaternion q;

    q.w = msg->pose.pose.orientation.w;
    q.x = msg->pose.pose.orientation.x;
    q.y = msg->pose.pose.orientation.y;
    q.z = msg->pose.pose.orientation.z;

    EulerAngles angles = ToEulerAngles(q);

    current_pos_.x = msg->pose.pose.position.x;
    current_pos_.y = msg->pose.pose.position.y;
    current_pos_.theta = angles.yaw * (180 / M_PI);
  }

  void laser_scan_callback(const LaserScan::SharedPtr msg) {
    laser_scan_msg_ = msg;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const robot_patrol::action::GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Received goal request with order");
    (void)uuid;
    desired_pos_ = goal->goal_pos;
    service_called = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;

    service_called = false;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    service_called = true;
    const auto goal = goal_handle->get_goal();
    auto feedback =
        std::make_shared<robot_patrol::action::GoToPose::Feedback>();
    auto &current_pos = feedback->current_pos;
    auto result = std::make_shared<robot_patrol::action::GoToPose::Result>();

    desired_pos_ = goal->goal_pos;

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

  void control_loop() {
    if (!service_called)
      return;

    Twist cmd_vel_msg;

    float dist_x_pos = this->desired_pos_.x - this->current_pos_.x;
    float dist_y_pos = this->desired_pos_.y - this->current_pos_.y;

    float target_rotation = atan2(dist_y_pos, dist_x_pos);
    float tolerence = 0.005;

    float angle_to_rotate =
        target_rotation - (this->current_pos_.theta * M_PI / 180.0);

    RCLCPP_INFO(this->get_logger(), "Angle to rotate: %f", angle_to_rotate);

    if (angle_to_rotate > M_PI)
      angle_to_rotate -= 2 * M_PI;

    if (angle_to_rotate < -M_PI)
      angle_to_rotate += 2 * M_PI;

    cmd_vel_msg.angular.z = 0.0;

    if (fabs(angle_to_rotate) > tolerence) {
      if (angle_to_rotate < 0)
        cmd_vel_msg.angular.z = -0.2;

      if (angle_to_rotate > 0)
        cmd_vel_msg.angular.z = 0.2;
    } else {
      service_called = false;
      RCLCPP_INFO(this->get_logger(), "Goal has been reached successfully");
    }

    cmd_vel_pub_->publish(cmd_vel_msg);
  }

  EulerAngles ToEulerAngles(Quaternion q) {
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = std::sqrt(1 + 2 * (q.w * q.y - q.x * q.z));
    double cosp = std::sqrt(1 - 2 * (q.w * q.y - q.x * q.z));
    angles.pitch = 2 * std::atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = std::atan2(siny_cosp, cosy_cosp);

    return angles;
  }

  bool goal_reached = false;
  bool service_called = false;

  Pose2D current_pos_;
  Pose2D desired_pos_;

  LaserScan::SharedPtr laser_scan_msg_;

  rclcpp::TimerBase::SharedPtr control_loop_timer_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<LaserScan>::SharedPtr laser_scan_sub_;
  rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp_action::Server<robot_patrol::action::GoToPose>::SharedPtr
      go_to_pose_service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoToPose>());
  rclcpp::shutdown();
}