#ifndef CIRCUIT_ROUTE_PLANNER__CIRCUIT_ROUTE_PLANNER_NODE_HPP_
#define CIRCUIT_ROUTE_PLANNER__CIRCUIT_ROUTE_PLANNER_NODE_HPP_

#include <autoware/route_handler/route_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include <autoware_map_msgs/msg/lanelet_map_bin.hpp>
#include <autoware_planning_msgs/msg/lanelet_primitive.hpp>
#include <autoware_planning_msgs/msg/lanelet_route.hpp>
#include <autoware_planning_msgs/msg/lanelet_segment.hpp>
#include <autoware_planning_msgs/msg/route_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <lanelet2_core/primitives/Lanelet.h>

#include <random>
#include <set>
#include <vector>

namespace autoware::circuit_route_planner
{

using autoware_map_msgs::msg::LaneletMapBin;
using autoware_planning_msgs::msg::LaneletPrimitive;
using autoware_planning_msgs::msg::LaneletRoute;
using autoware_planning_msgs::msg::LaneletSegment;
using autoware_planning_msgs::msg::RouteState;
using geometry_msgs::msg::Pose;
using nav_msgs::msg::Odometry;
using unique_identifier_msgs::msg::UUID;

class CircuitRoutePlannerNode : public rclcpp::Node
{
public:
  explicit CircuitRoutePlannerNode(const rclcpp::NodeOptions & options);

private:
  void onMap(const LaneletMapBin::ConstSharedPtr & msg);
  void onOdometry(const Odometry::ConstSharedPtr & msg);
  void onTimer();

  bool buildRoute(const Pose & current_pose, const lanelet::ConstLanelet & start_ll,
    LaneletRoute & route_out);

  static double laneletLength(const lanelet::ConstLanelet & ll);
  static Pose poseAtCenterlineEnd(const lanelet::ConstLanelet & ll);
  static UUID generateUUID();

  rclcpp::Subscription<LaneletMapBin>::SharedPtr map_sub_;
  rclcpp::Subscription<Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<LaneletRoute>::SharedPtr route_pub_;
  rclcpp::Publisher<RouteState>::SharedPtr route_state_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  autoware::route_handler::RouteHandler route_handler_;
  bool is_map_ready_{false};

  Odometry::ConstSharedPtr current_odom_;
  lanelet::ConstLanelets road_lanelets_;

  lanelet::Id last_closest_id_{-1};
  UUID persistent_uuid_;  // fixed for the lifetime of this node; avoids reset() on each update

  double lookahead_distance_m_{30.0};
  int backward_lanelets_num_{2};  // lanelets behind car included in route for overlap continuity
};

}  // namespace autoware::circuit_route_planner

#endif  // CIRCUIT_ROUTE_PLANNER__CIRCUIT_ROUTE_PLANNER_NODE_HPP_
