#ifndef ROBORACER_INTERFACE__ROBORACER_INTERFACE_NODE_HPP_
#define ROBORACER_INTERFACE__ROBORACER_INTERFACE_NODE_HPP_

#include <deque>
#include <string>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_vehicle_msgs/msg/control_mode_report.hpp"
#include "autoware_vehicle_msgs/msg/steering_report.hpp"
#include "autoware_vehicle_msgs/msg/velocity_report.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

class RoboracerInterfaceNode : public rclcpp::Node
{
public:
  RoboracerInterfaceNode();

private:
  void onControlCmd(const autoware_control_msgs::msg::Control::SharedPtr msg);
  void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg);
  void onJoy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void onServoPosition(const std_msgs::msg::Float64::SharedPtr msg);
  void publishSteeringReport();
  void publishVelocityReport();

  // Returns the updated moving average after pushing a new sample.
  double updateMovingAverage(std::deque<double> & window, double & sum, double sample) const;

  // Rounds to the given number of decimal places if >= 0, otherwise returns value unchanged.
  static double maybeRound(double value, int decimal_places);

  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr control_cmd_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr servo_position_sub_;
  rclcpp::TimerBase::SharedPtr steering_report_timer_;
  rclcpp::TimerBase::SharedPtr velocity_report_timer_;

  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlModeReport>::SharedPtr control_mode_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::SteeringReport>::SharedPtr steering_status_pub_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::VelocityReport>::SharedPtr velocity_status_pub_;

  double steering_report_rate_hz_{30.0};
  float current_steering_angle_{0.0F};
  float current_longitudinal_velocity_{0.0F};
  float current_lateral_velocity_{0.0F};
  float current_heading_rate_{0.0F};

  int autonomous_button_{6};
  int manual_button_{7};

  int moving_average_window_{10};
  int longitudinal_decimal_places_{1};  // -1 disables rounding
  int lateral_decimal_places_{2};
  int heading_rate_decimal_places_{3};

  // Moving average state: one window + running sum per velocity channel
  std::deque<double> long_vel_window_;
  std::deque<double> lat_vel_window_;
  std::deque<double> heading_rate_window_;
  double long_vel_sum_{0.0};
  double lat_vel_sum_{0.0};
  double heading_rate_sum_{0.0};
};

#endif  // ROBORACER_INTERFACE__ROBORACER_INTERFACE_NODE_HPP_
