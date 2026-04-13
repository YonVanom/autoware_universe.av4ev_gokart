#include <functional>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

class IsaacOdomToPoseNode : public rclcpp::Node
{
public:
  IsaacOdomToPoseNode()
  : Node("isaac_odom_to_pose_node")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/ego/odom");
    const auto output_pose_topic = declare_parameter<std::string>(
      "output_pose_topic", "/sensing/gnss/pose");
    const auto output_pose_with_cov_topic = declare_parameter<std::string>(
      "output_pose_with_covariance_topic", "/sensing/gnss/pose_with_covariance");

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic, rclcpp::SensorDataQoS(),
      std::bind(&IsaacOdomToPoseNode::onOdometry, this, std::placeholders::_1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      output_pose_topic, rclcpp::QoS(1));
    pose_with_cov_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      output_pose_with_cov_topic, rclcpp::QoS(1));

    RCLCPP_INFO(
      get_logger(), "Converting '%s' -> '%s' and '%s'",
      input_topic.c_str(), output_pose_topic.c_str(), output_pose_with_cov_topic.c_str());
  }

private:
  void onOdometry(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    auto pose = msg->pose.pose;
    
    // Enforce positive scalar part (w >= 0)
    if (pose.orientation.w < 0.0) {
      pose.orientation.x *= -1.0;
      pose.orientation.y *= -1.0;
      pose.orientation.z *= -1.0;
      pose.orientation.w *= -1.0;
    }
      
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = "map";
    pose_msg.pose = pose;
    pose_pub_->publish(pose_msg);

    geometry_msgs::msg::PoseWithCovarianceStamped pose_with_cov_msg;
    pose_with_cov_msg.header = msg->header;
    pose_with_cov_msg.header.frame_id = "map";
    pose_with_cov_msg.pose = msg->pose;
    pose_with_cov_msg.pose.pose = pose;
    pose_with_cov_pub_->publish(pose_with_cov_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_with_cov_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IsaacOdomToPoseNode>());
  rclcpp::shutdown();
  return 0;
}
