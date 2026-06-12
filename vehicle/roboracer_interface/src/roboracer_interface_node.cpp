#include <cmath>
#include <memory>

#include "roboracer_interface/roboracer_interface_node.hpp"

RoboracerInterfaceNode::RoboracerInterfaceNode()
: Node("roboracer_interface_node")
{
  using std::placeholders::_1;

  steering_report_rate_hz_ =
    this->declare_parameter<double>("steering_report_rate_hz", steering_report_rate_hz_);
  autonomous_button_ =
    this->declare_parameter<int>("autonomous_button", autonomous_button_);
  manual_button_ =
    this->declare_parameter<int>("manual_button", manual_button_);
  moving_average_window_ =
    this->declare_parameter<int>("moving_average_window", moving_average_window_);
  longitudinal_decimal_places_ =
    this->declare_parameter<int>("longitudinal_decimal_places", longitudinal_decimal_places_);
  lateral_decimal_places_ =
    this->declare_parameter<int>("lateral_decimal_places", lateral_decimal_places_);
  heading_rate_decimal_places_ =
    this->declare_parameter<int>("heading_rate_decimal_places", heading_rate_decimal_places_);

  control_cmd_sub_ = this->create_subscription<autoware_control_msgs::msg::Control>(
    "control/command/control_cmd", rclcpp::QoS{1},
    std::bind(&RoboracerInterfaceNode::onControlCmd, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "ego/odom", rclcpp::QoS{1},
    std::bind(&RoboracerInterfaceNode::onOdom, this, _1));

  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "joy", rclcpp::QoS{1},
    std::bind(&RoboracerInterfaceNode::onJoy, this, _1));

  servo_position_sub_ = this->create_subscription<std_msgs::msg::Float64>(
    "commands/servo/position", rclcpp::QoS{1},
    std::bind(&RoboracerInterfaceNode::onServoPosition, this, _1));

  drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
    "ego/drive", rclcpp::QoS{1});

  control_mode_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::ControlModeReport>(
    "vehicle/status/control_mode", rclcpp::QoS{1});

  steering_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::SteeringReport>(
    "vehicle/status/steering_status", rclcpp::QoS{1});

  velocity_status_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::VelocityReport>(
    "vehicle/status/velocity_status", rclcpp::QoS{1});

  const auto period =
    std::chrono::duration<double>(1.0 / steering_report_rate_hz_);
  steering_report_timer_ = this->create_wall_timer(
    period, std::bind(&RoboracerInterfaceNode::publishSteeringReport, this));
  velocity_report_timer_ = this->create_wall_timer(
    period, std::bind(&RoboracerInterfaceNode::publishVelocityReport, this));
}

double RoboracerInterfaceNode::updateMovingAverage(
  std::deque<double> & window, double & sum, double sample) const
{
  sum += sample;
  window.push_back(sample);
  if (static_cast<int>(window.size()) > moving_average_window_) {
    sum -= window.front();
    window.pop_front();
  }
  return sum / static_cast<double>(window.size());
}

double RoboracerInterfaceNode::maybeRound(double value, int decimal_places)
{
  if (decimal_places < 0) {
    return value;
  }
  const double scale = std::pow(10.0, decimal_places);
  return std::round(value * scale) / scale;
}

void RoboracerInterfaceNode::onControlCmd(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = msg->stamp;
  drive.drive.speed = msg->longitudinal.velocity;
  drive.drive.steering_angle = msg->lateral.steering_tire_angle;
  drive_pub_->publish(drive);
}

void RoboracerInterfaceNode::publishSteeringReport()
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = this->now();
  steering.steering_tire_angle = current_steering_angle_;
  steering_status_pub_->publish(steering);
}

void RoboracerInterfaceNode::onServoPosition(const std_msgs::msg::Float64::SharedPtr msg)
{
  current_steering_angle_ = static_cast<float>(msg->data);
}

void RoboracerInterfaceNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_longitudinal_velocity_ = static_cast<float>(maybeRound(
    updateMovingAverage(long_vel_window_, long_vel_sum_, msg->twist.twist.linear.x),
    longitudinal_decimal_places_));
  current_lateral_velocity_ = static_cast<float>(maybeRound(
    updateMovingAverage(lat_vel_window_, lat_vel_sum_, msg->twist.twist.linear.y),
    lateral_decimal_places_));
  current_heading_rate_ = static_cast<float>(maybeRound(
    updateMovingAverage(heading_rate_window_, heading_rate_sum_, msg->twist.twist.angular.z),
    heading_rate_decimal_places_));
}

void RoboracerInterfaceNode::publishVelocityReport()
{
  autoware_vehicle_msgs::msg::VelocityReport velocity;
  velocity.header.stamp = this->now();
  velocity.header.frame_id = "base_link";
  velocity.longitudinal_velocity = current_longitudinal_velocity_;
  velocity.lateral_velocity = current_lateral_velocity_;
  velocity.heading_rate = current_heading_rate_;
  velocity_status_pub_->publish(velocity);
}

void RoboracerInterfaceNode::onJoy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::ControlModeReport out;
  out.stamp = this->now();

  const auto n = static_cast<int>(msg->buttons.size());
  if (autonomous_button_ < n && msg->buttons[autonomous_button_]) {
    out.mode = autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS;
  } else if (manual_button_ < n && msg->buttons[manual_button_]) {
    out.mode = autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
  } else {
    out.mode = autoware_vehicle_msgs::msg::ControlModeReport::NO_COMMAND;
  }

  control_mode_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboracerInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
