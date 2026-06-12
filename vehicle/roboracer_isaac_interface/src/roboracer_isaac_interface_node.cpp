#include <cmath>
#include <memory>

#include "roboracer_isaac_interface/roboracer_isaac_interface_node.hpp"

RoboracerIsaacInterfaceNode::RoboracerIsaacInterfaceNode()
: Node("roboracer_isaac_interface_node")
{
  using std::placeholders::_1;

  steering_report_rate_hz_ =
    this->declare_parameter<double>("steering_report_rate_hz", steering_report_rate_hz_);
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
    std::bind(&RoboracerIsaacInterfaceNode::onControlCmd, this, _1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "ego/odom", rclcpp::QoS{1},
    std::bind(&RoboracerIsaacInterfaceNode::onOdom, this, _1));

  control_mode_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "ego/control_mode", rclcpp::QoS{1},
    std::bind(&RoboracerIsaacInterfaceNode::onControlMode, this, _1));

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
    period, std::bind(&RoboracerIsaacInterfaceNode::publishSteeringReport, this));
}

double RoboracerIsaacInterfaceNode::updateMovingAverage(
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

double RoboracerIsaacInterfaceNode::maybeRound(double value, int decimal_places)
{
  if (decimal_places < 0) {
    return value;
  }
  const double scale = std::pow(10.0, decimal_places);
  return std::round(value * scale) / scale;
}

void RoboracerIsaacInterfaceNode::onControlCmd(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  current_steering_angle_ = msg->lateral.steering_tire_angle;

  ackermann_msgs::msg::AckermannDriveStamped drive;
  drive.header.stamp = msg->stamp;
  drive.drive.speed = msg->longitudinal.velocity;
  drive.drive.steering_angle = msg->lateral.steering_tire_angle;
  drive_pub_->publish(drive);
}

void RoboracerIsaacInterfaceNode::publishSteeringReport()
{
  autoware_vehicle_msgs::msg::SteeringReport steering;
  steering.stamp = this->now();
  steering.steering_tire_angle = current_steering_angle_;
  steering_status_pub_->publish(steering);
}

void RoboracerIsaacInterfaceNode::onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  const double long_vel = maybeRound(
    updateMovingAverage(long_vel_window_, long_vel_sum_, msg->twist.twist.linear.x),
    longitudinal_decimal_places_);
  const double lat_vel = maybeRound(
    updateMovingAverage(lat_vel_window_, lat_vel_sum_, msg->twist.twist.linear.y),
    lateral_decimal_places_);
  const double heading_rate = maybeRound(
    updateMovingAverage(heading_rate_window_, heading_rate_sum_, msg->twist.twist.angular.z),
    heading_rate_decimal_places_);

  autoware_vehicle_msgs::msg::VelocityReport velocity;
  velocity.header = msg->header;
  velocity.header.frame_id = "base_link";  // Override frame to match expected value in Autoware
  velocity.longitudinal_velocity = static_cast<float>(long_vel);
  velocity.lateral_velocity = static_cast<float>(lat_vel);
  velocity.heading_rate = static_cast<float>(heading_rate);
  velocity_status_pub_->publish(velocity);
}

void RoboracerIsaacInterfaceNode::onControlMode(const std_msgs::msg::Int32::SharedPtr msg)
{
  autoware_vehicle_msgs::msg::ControlModeReport out;
  out.stamp = this->now();
  out.mode = (msg->data == 1)
    ? autoware_vehicle_msgs::msg::ControlModeReport::AUTONOMOUS
    : autoware_vehicle_msgs::msg::ControlModeReport::MANUAL;
  control_mode_pub_->publish(out);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoboracerIsaacInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
