//  Copyright 2022 Tier IV, Inc. All rights reserved.
//
//  Licensed under the Apache License, Version 2.0 (the "License");
//  you may not use this file except in compliance with the License.
//  You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software
//  distributed under the License is distributed on an "AS IS" BASIS,
//  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  See the License for the specific language governing permissions and
//  limitations under the License.

#include "trajectory_follower_f1tenth/f1tenth_trajectory_follower.hpp"

#include <autoware/motion_utils/trajectory/trajectory.hpp>
#include <iostream>
#include <algorithm>

using namespace std;

namespace f1tenth_trajectory_follower
{

using autoware::motion_utils::findNearestIndex;

F1tenthTrajectoryFollower::F1tenthTrajectoryFollower(const rclcpp::NodeOptions & options)
: Node("f1tenth_trajectory_follower", options)
{
  drive_cmd_ = create_publisher<AckermannDriveStamped>("/drive", 1);
  traj_marker_pub_ = create_publisher<Marker>("/wp_marker", 1);
  goal_marker_pub_ = create_publisher<Marker>("/goal_marker", 1);

  sub_kinematics_ = create_subscription<Odometry>(
    "input/kinematics", 1, [this](const Odometry::SharedPtr msg) { odometry_ = msg; });
  sub_trajectory_ = create_subscription<AutowarePlanningMsgs::Trajectory>(
    "input/trajectory", 1, [this](const AutowarePlanningMsgs::Trajectory::SharedPtr msg) { 
    autoware_auto_trajectory_ = msg; 
    convertTrajectoryMsg();});

  use_external_target_vel_ = declare_parameter<bool>("use_external_target_vel", false);
  external_target_vel_ = declare_parameter<float>("external_target_vel", 0.0);
  lateral_deviation_ = declare_parameter<float>("lateral_deviation", 0.0);

  using namespace std::literals::chrono_literals;
  timer_ = rclcpp::create_timer(
    this, get_clock(), 30ms, std::bind(&F1tenthTrajectoryFollower::onTimer, this));
}

void F1tenthTrajectoryFollower::createTrajectoryMarker(){
    scale.x = 0.1;
    scale.y = 0.1;
    scale.z = 0.1;

    header.frame_id = "map";

    marker.type = marker.POINTS;
    marker.pose = pose;
    marker.scale = scale;
    marker.header = header;
    marker.points.clear();
    marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);

    for(int i = 0; i < (int)trajectory_.points.size(); i++){
      point.x = trajectory_.points[i].pose.position.x;
      point.y = trajectory_.points[i].pose.position.y;
      point.z = 0.0;
      marker.points.push_back(point);
    }

    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    traj_marker_pub_->publish(marker);

    scale.x = 0.2;
    scale.y = 0.2;
    scale.z = 0.2;

    goal_marker.type = goal_marker.POINTS;
    goal_marker.pose = pose;
    goal_marker.scale = scale;
    goal_marker.header = header;
    goal_marker.lifetime = rclcpp::Duration::from_nanoseconds(0.03 * 1e9);

    point.x = lookahead_traj_point_.pose.position.x;
    point.y = lookahead_traj_point_.pose.position.y;
    point.z = 0.0;
    
    goal_marker.points.clear();
    goal_marker.points.push_back(point);

    goal_marker.color.r = 0.0;
    goal_marker.color.g = 0.0;
    goal_marker.color.b = 1.0;
    goal_marker.color.a = 1.0;

    goal_marker_pub_->publish(goal_marker);
}

// convert trajectory from autoware_auto_planning_msgs to autoware_planning_msgs
void F1tenthTrajectoryFollower::convertTrajectoryMsg(){
  size_t num_points = autoware_auto_trajectory_->points.size();
  AutowarePlanningMsgs::TrajectoryPoint* trajectory_points = 
    new AutowarePlanningMsgs::TrajectoryPoint[num_points];
  
  for(int i = 0; i < (int)num_points; i++){
    AutowarePlanningMsgs::TrajectoryPoint& point = trajectory_points[i];
    AutowarePlanningMsgs::TrajectoryPoint& auto_point = autoware_auto_trajectory_->points[i];
    point.time_from_start = auto_point.time_from_start;
    point.pose = auto_point.pose;
    point.longitudinal_velocity_mps = auto_point.longitudinal_velocity_mps;
    point.lateral_velocity_mps = auto_point.lateral_velocity_mps;
    point.acceleration_mps2 = auto_point.acceleration_mps2;
    point.heading_rate_rps = auto_point.heading_rate_rps;
    point.front_wheel_angle_rad = auto_point.front_wheel_angle_rad;
    point.rear_wheel_angle_rad = auto_point.rear_wheel_angle_rad;
  }
  trajectory_.points.assign(trajectory_points, trajectory_points + num_points);
}

void F1tenthTrajectoryFollower::onTimer()
{
  if (!checkData()) {
    // RCLCPP_INFO(get_logger(), "data not ready!!");
    return;
  }

  updateClosest();
  createTrajectoryMarker();

  Control cmd;
  cmd.stamp = cmd.lateral.stamp = cmd.longitudinal.stamp = get_clock()->now();
  cmd.lateral.steering_tire_angle = static_cast<float>(calcSteerCmd());
  cmd.longitudinal.velocity = use_external_target_vel_ ? static_cast<float>(external_target_vel_)
                                                    : closest_traj_point_.longitudinal_velocity_mps;
  cmd.longitudinal.acceleration = static_cast<float>(calcAccCmd());

  ackermann_msgs::msg::AckermannDriveStamped ackermann_msg;
  ackermann_msg.drive.speed = cmd.longitudinal.velocity;
  ackermann_msg.drive.steering_angle = cmd.lateral.steering_tire_angle;
  drive_cmd_->publish(ackermann_msg);

  cout << "velocity: " << ackermann_msg.drive.speed << "m/s"<< endl;
}

void F1tenthTrajectoryFollower::updateClosest()
{
  closest_idx_ = findNearestIndex(trajectory_.points, odometry_->pose.pose.position);
  closest_traj_point_ = trajectory_.points.at(closest_idx_);

  constexpr auto lookahead_time = 3.0;
  constexpr auto min_lookahead = 3.0;
  const auto lookahead = min_lookahead + lookahead_time * std::abs(odometry_->twist.twist.linear.x);

  const auto & ego_pos = odometry_->pose.pose.position;
  size_t lookahead_idx = closest_idx_;
  for (size_t i = closest_idx_; i < trajectory_.points.size(); ++i) {
    const auto dx = trajectory_.points[i].pose.position.x - ego_pos.x;
    const auto dy = trajectory_.points[i].pose.position.y - ego_pos.y;
    if (std::sqrt(dx * dx + dy * dy) >= lookahead) {
      lookahead_idx = i;
      break;
    }
    lookahead_idx = i;
  }
  lookahead_traj_point_ = trajectory_.points.at(lookahead_idx);
}

double F1tenthTrajectoryFollower::calcSteerCmd()
{
  constexpr auto wheel_base = 0.325;
  constexpr auto steer_lim = 1.0;

  // Extract ego yaw from quaternion
  const auto & ego_pos = odometry_->pose.pose.position;
  const auto & q = odometry_->pose.pose.orientation;
  const auto ego_yaw = std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));

  // Transform lookahead point from map frame into vehicle frame
  const auto dx = lookahead_traj_point_.pose.position.x - ego_pos.x;
  const auto dy = lookahead_traj_point_.pose.position.y - ego_pos.y;
  const auto x_veh =  std::cos(ego_yaw) * dx + std::sin(ego_yaw) * dy;
  const auto y_veh = -std::sin(ego_yaw) * dx + std::cos(ego_yaw) * dy + lateral_deviation_;

  // Geometric pure pursuit: steer = atan(2L * sin(alpha) / ld)
  // sin(alpha) = y_veh / ld  =>  steer = atan2(2L * y_veh, ld^2)
  const auto ld_sq = x_veh * x_veh + y_veh * y_veh;
  if (ld_sq < 1e-6) return 0.0;

  const auto steer = std::atan2(2.0 * wheel_base * y_veh, ld_sq);
  return std::clamp(steer, -steer_lim, steer_lim);
}

double F1tenthTrajectoryFollower::calcAccCmd()
{
  const auto traj_vel = static_cast<double>(closest_traj_point_.longitudinal_velocity_mps);
  const auto ego_vel = odometry_->twist.twist.linear.x;
  const auto target_vel = use_external_target_vel_ ? external_target_vel_ : traj_vel;
  const auto vel_err = ego_vel - target_vel;

  // P feedback
  constexpr auto kp = 0.5;
  constexpr auto acc_lim = 2.0;

  const auto acc = std::clamp(-kp * vel_err, -acc_lim, acc_lim);
  RCLCPP_DEBUG(get_logger(), "vel_err = %f, acc = %f", vel_err, acc);
  return acc;
}

bool F1tenthTrajectoryFollower::checkData() { return (autoware_auto_trajectory_ && odometry_); }

}  // namespace f1tenth_trajectory_follower

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(f1tenth_trajectory_follower::F1tenthTrajectoryFollower)
