#include "circuit_route_planner_node.hpp"

#include <autoware_lanelet2_extension/utility/query.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace autoware::circuit_route_planner
{

CircuitRoutePlannerNode::CircuitRoutePlannerNode(const rclcpp::NodeOptions & options)
: Node("circuit_route_planner", options)
{
  lookahead_distance_m_ = declare_parameter<double>("lookahead_distance_m", 30.0);
  backward_lanelets_num_ = declare_parameter<int>("backward_lanelets_num", 2);
  persistent_uuid_ = generateUUID();

  const auto qos_transient = rclcpp::QoS{1}.transient_local();

  map_sub_ = create_subscription<LaneletMapBin>(
    "~/input/vector_map", qos_transient,
    std::bind(&CircuitRoutePlannerNode::onMap, this, std::placeholders::_1));

  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", rclcpp::QoS{1},
    std::bind(&CircuitRoutePlannerNode::onOdometry, this, std::placeholders::_1));

  route_pub_ = create_publisher<LaneletRoute>("~/output/route", qos_transient);
  route_state_pub_ = create_publisher<RouteState>("~/output/route_state", qos_transient);

  timer_ = create_wall_timer(
    std::chrono::milliseconds(200),
    std::bind(&CircuitRoutePlannerNode::onTimer, this));
}

void CircuitRoutePlannerNode::onMap(const LaneletMapBin::ConstSharedPtr & msg)
{
  RCLCPP_INFO(get_logger(), "Map received. Building routing graph.");
  route_handler_.setMap(*msg);
  const auto all_lls = lanelet::utils::query::laneletLayer(route_handler_.getLaneletMapPtr());
  road_lanelets_ = lanelet::utils::query::roadLanelets(all_lls);
  is_map_ready_ = true;
  last_closest_id_ = -1;
  RCLCPP_INFO(get_logger(), "Map ready. Found %zu road lanelets.", road_lanelets_.size());
}

void CircuitRoutePlannerNode::onOdometry(const Odometry::ConstSharedPtr & msg)
{
  current_odom_ = msg;
}

void CircuitRoutePlannerNode::onTimer()
{
  if (!is_map_ready_ || !current_odom_) return;

  const auto & pose = current_odom_->pose.pose;

  lanelet::ConstLanelet closest;
  if (!lanelet::utils::query::getClosestLanelet(road_lanelets_, pose, &closest)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Could not find closest lanelet to current pose.");
    return;
  }

  // Replan when the car enters a new lanelet
  if (closest.id() == last_closest_id_) return;

  LaneletRoute route;
  if (!buildRoute(pose, closest, route)) {
    RCLCPP_WARN(get_logger(), "Failed to build route from lanelet %ld.", closest.id());
    return;
  }

  route_pub_->publish(route);

  RouteState route_state;
  route_state.stamp = now();
  route_state.state = RouteState::SET;
  route_state_pub_->publish(route_state);

  last_closest_id_ = closest.id();
  RCLCPP_INFO(get_logger(), "Published route: %zu segments from lanelet %ld (+%d back).",
    route.segments.size(), closest.id(), backward_lanelets_num_);
}

bool CircuitRoutePlannerNode::buildRoute(
  const Pose & current_pose, const lanelet::ConstLanelet & closest_ll,
  LaneletRoute & route_out)
{
  // --- collect backward lanelets (already-driven portion for overlap) ---
  std::vector<lanelet::ConstLanelet> backward_lls;
  {
    auto cur = closest_ll;
    for (int i = 0; i < backward_lanelets_num_; ++i) {
      const auto prevs = route_handler_.getPreviousLanelets(cur);
      if (prevs.empty()) break;
      cur = prevs.front();
      backward_lls.push_back(cur);
    }
    // collected in reverse order (closest-first), reverse to get oldest-first
    std::reverse(backward_lls.begin(), backward_lls.end());
  }

  // --- collect forward lanelets up to lookahead_distance ---
  // visited tracks only forward-traversal to detect circuit loops.
  // backward_ids is used to skip duplicates without seeding visited — seeding visited would
  // cause forward collection to break immediately on compact circuits where the backward
  // window wraps around close to closest_ll.
  std::set<lanelet::Id> backward_ids;
  for (const auto & ll : backward_lls) backward_ids.insert(ll.id());

  std::vector<lanelet::ConstLanelet> forward_lls;
  {
    std::set<lanelet::Id> visited;
    double accumulated = 0.0;
    auto cur = closest_ll;
    while (accumulated < lookahead_distance_m_) {
      if (visited.count(cur.id())) break;  // loop detected in forward direction
      visited.insert(cur.id());
      accumulated += laneletLength(cur);
      if (!backward_ids.count(cur.id())) {
        forward_lls.push_back(cur);  // omit lanelets already covered by backward window
      }

      const auto nexts = route_handler_.getNextLanelets(cur);
      if (nexts.empty()) break;
      cur = nexts.front();
    }
  }

  if (forward_lls.empty()) return false;

  const auto & last_ll = forward_lls.back();
  if (last_ll.centerline().size() < 2) return false;
  const Pose goal_pose = poseAtCenterlineEnd(last_ll);

  // --- build segment list: backward + forward ---
  std::vector<LaneletSegment> segments;
  segments.reserve(backward_lls.size() + forward_lls.size());

  auto make_segment = [](const lanelet::ConstLanelet & ll) {
    LaneletPrimitive prim;
    prim.id = ll.id();
    prim.primitive_type = "lane";
    LaneletSegment seg;
    seg.preferred_primitive = prim;
    seg.primitives.push_back(prim);
    return seg;
  };

  for (const auto & ll : backward_lls) segments.push_back(make_segment(ll));
  for (const auto & ll : forward_lls) segments.push_back(make_segment(ll));

  route_out.header.stamp = now();
  route_out.header.frame_id = "map";
  route_out.start_pose = current_pose;
  route_out.goal_pose = goal_pose;
  route_out.segments = segments;
  route_out.uuid = persistent_uuid_;
  route_out.allow_modification = false;

  return true;
}

double CircuitRoutePlannerNode::laneletLength(const lanelet::ConstLanelet & ll)
{
  double len = 0.0;
  const auto & cl = ll.centerline();
  for (size_t i = 1; i < cl.size(); ++i) {
    const double dx = cl[i].x() - cl[i - 1].x();
    const double dy = cl[i].y() - cl[i - 1].y();
    len += std::sqrt(dx * dx + dy * dy);
  }
  return len;
}

Pose CircuitRoutePlannerNode::poseAtCenterlineEnd(const lanelet::ConstLanelet & ll)
{
  const auto & cl = ll.centerline();
  const auto & p_end = cl.back();
  const auto & p_prev = cl[cl.size() - 2];

  const double yaw = std::atan2(
    p_end.y() - p_prev.y(),
    p_end.x() - p_prev.x());

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);

  Pose pose;
  pose.position.x = p_end.x();
  pose.position.y = p_end.y();
  pose.position.z = p_end.z();
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return pose;
}

UUID CircuitRoutePlannerNode::generateUUID()
{
  static std::independent_bits_engine<std::mt19937, 8, uint8_t> engine(std::random_device{}());
  UUID uuid;
  for (auto & byte : uuid.uuid) byte = engine();
  return uuid;
}

}  // namespace autoware::circuit_route_planner

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(autoware::circuit_route_planner::CircuitRoutePlannerNode)
