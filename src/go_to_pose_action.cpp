#include "robot_patrol/action/go_to_pose.hpp"
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
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

enum class RobotState {
  IDLE,
  ROTATE,
  MOVE,
  POSITION,
  COMPLETE,
  OBSTACLE_DETECTED
};

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
    control_loop_timer_ = this->create_wall_timer(
        100ms, std::bind(&GoToPose::control_loop, this));
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

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");
  }

private:
  struct EulerAngles {
    double roll, pitch, yaw;
  };

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
    laser_scan_ranges = msg->ranges;
    front_ray = laser_scan_ranges.size() / 2;
    front_ray_view = (30.0 * M_PI / 180.0) / msg->angle_increment;
  }

  rclcpp_action::GoalResponse handle_goal(
      const rclcpp_action::GoalUUID &uuid,
      std::shared_ptr<const robot_patrol::action::GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(), "Action Server Called");

    (void)uuid;
    this->desired_pos_ = goal->goal_pos;
    this->service_called = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Action Server Cancelled");

    (void)goal_handle;
    service_called = false;

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
    rclcpp::Rate loop_rate(1);

    const auto goal = goal_handle->get_goal();
    auto result = std::make_shared<robot_patrol::action::GoToPose::Result>();
    auto feedback =
        std::make_shared<robot_patrol::action::GoToPose::Feedback>();
    auto &current_pos = feedback->current_pos;
    bool aborted = false;

    desired_pos_ = goal->goal_pos;

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
      return;
    }

    while (robot_state_ != RobotState::COMPLETE) {
      if (robot_state_ == RobotState::OBSTACLE_DETECTED) {
        result->status = false;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(),
                    "Action Server Error - Obstacle detected in front");
        aborted = true;
        break;
      }

      current_pos = current_pos_;
      goal_handle->publish_feedback(feedback);
      loop_rate.sleep();
    }

    if (!aborted) {
      result->status = true;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Action Server Complete");
    }

    robot_state_ = RobotState::IDLE;
    service_called = false;
  }

  void control_loop() {
    switch (robot_state_) {
    case RobotState::IDLE:
      if (service_called)
        robot_state_ = RobotState::ROTATE;
      break;
    case RobotState::ROTATE:
      rotate_toward_goal();
      break;
    case RobotState::MOVE:
      move_toward_goal();
      break;
    case RobotState::POSITION:
      position_toward_goal();
      break;
    }

    cmd_vel_pub_->publish(cmd_vel_msg_);
  }

  void rotate_toward_goal() {
    float dist_x_pos = this->desired_pos_.x - this->current_pos_.x;
    float dist_y_pos = this->desired_pos_.y - this->current_pos_.y;
    float target_rotation = atan2(dist_y_pos, dist_x_pos);
    float angle_to_rotate =
        target_rotation - (this->current_pos_.theta * M_PI / 180.0);
    float tolerence = 0.005;

    if (angle_to_rotate > M_PI)
      angle_to_rotate -= 2 * M_PI;

    if (angle_to_rotate < -M_PI)
      angle_to_rotate += 2 * M_PI;

    if (abs(angle_to_rotate) > tolerence) {
      if (angle_to_rotate < 0)
        cmd_vel_msg_.angular.z = -0.2;

      if (angle_to_rotate > 0)
        cmd_vel_msg_.angular.z = 0.2;
    } else {
      cmd_vel_msg_.angular.z = 0.0;
      robot_state_ = RobotState::MOVE;
    }
  }

  void move_toward_goal() {
    float distance_to_tavel = sqrt(pow(desired_pos_.x - current_pos_.x, 2) +
                                   pow(desired_pos_.y - current_pos_.y, 2));
    float tolerence = 0.1;

    for (int i = front_ray - front_ray_view; i < front_ray + front_ray_view;
         i++) {
      if (laser_scan_ranges[i] <= 0.35) {
        cmd_vel_msg_.linear.x = 0.0;
        robot_state_ = RobotState::OBSTACLE_DETECTED;
        return;
      }
    }

    if (distance_to_tavel > tolerence) {
      cmd_vel_msg_.linear.x = 0.2;
    } else {
      cmd_vel_msg_.linear.x = 0.0;
      robot_state_ = RobotState::POSITION;
    }
  }

  void position_toward_goal() {
    float tolerence = 1;
    float angle_diff = desired_pos_.theta - current_pos_.theta;

    if (angle_diff > 180)
      angle_diff -= 360;
    if (angle_diff < -180)
      angle_diff += 360;

    if (abs(angle_diff) > tolerence) {
      cmd_vel_msg_.angular.z = angle_diff > 0 ? 0.2 : -0.2;
    } else {
      cmd_vel_msg_.angular.z = 0.0;
      robot_state_ = RobotState::COMPLETE;
    }
  }

  RobotState robot_state_ = RobotState::IDLE;

  std::vector<float> laser_scan_ranges;
  int front_ray = 0;
  int front_ray_view = 0;
  bool service_called = false;

  Twist cmd_vel_msg_;
  Pose2D current_pos_;
  Pose2D desired_pos_;

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