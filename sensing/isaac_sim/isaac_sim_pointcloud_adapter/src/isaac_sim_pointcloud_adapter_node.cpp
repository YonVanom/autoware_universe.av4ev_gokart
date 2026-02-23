#include <algorithm>
#include <cstdint>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

class IsaacSimPointCloudAdapterNode : public rclcpp::Node
{
public:
  IsaacSimPointCloudAdapterNode()
  : Node("isaac_sim_pointcloud_adapter")
  {
    input_topic_ = declare_parameter<std::string>(
      "input_topic", "/sensing/lidar/top/pointcloud_raw");
    output_topic_ = declare_parameter<std::string>(
      "output_topic", "/sensing/lidar/top/pointcloud_raw_ex");

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&IsaacSimPointCloudAdapterNode::pointcloudCallback, this, std::placeholders::_1));

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic_,
      rclcpp::SensorDataQoS());

    RCLCPP_INFO(
      get_logger(),
      "Isaac Sim → Autoware pointcloud adapter started\n"
      "  input_topic:  %s\n"
      "  output_topic: %s",
      input_topic_.c_str(),
      output_topic_.c_str());
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = msg->height;
    out.width = msg->width;
    out.is_dense = msg->is_dense;
    out.is_bigendian = false;

    // Define Autoware PointXYZIRC layout
    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::UINT8,
      "return_type", 1, sensor_msgs::msg::PointField::UINT8,
      "channel", 1, sensor_msgs::msg::PointField::UINT16);

    modifier.resize(msg->width * msg->height);

    // ---- Input iterators (Isaac Sim fields) ----
    sensor_msgs::PointCloud2ConstIterator<float>    in_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float>    in_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float>    in_z(*msg, "z");
    sensor_msgs::PointCloud2ConstIterator<float>    in_intensity(*msg, "intensity");
    sensor_msgs::PointCloud2ConstIterator<uint32_t> in_channel(*msg, "channel_id");
    sensor_msgs::PointCloud2ConstIterator<uint8_t>  in_return(*msg, "echo_id");

    // ---- Output iterators (Autoware layout) ----
    sensor_msgs::PointCloud2Iterator<float>    out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float>    out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float>    out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t>  out_intensity(out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t>  out_return(out, "return_type");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_channel(out, "channel");

    for (; in_x != in_x.end();
         ++in_x, ++in_y, ++in_z,
         ++in_intensity, ++in_return, ++in_channel,
         ++out_x, ++out_y, ++out_z,
         ++out_intensity, ++out_return, ++out_channel)
    {
      *out_x = *in_x;
      *out_y = *in_y;
      *out_z = *in_z;

      // Convert intensity → uint8 (clamped)
      float intensity = *in_intensity;
      intensity = std::clamp(intensity, 0.0f, 255.0f);
      *out_intensity = static_cast<uint8_t>(intensity);

      // Convert channel → uint16 (clamped)
      uint32_t channel = *in_channel;
      channel = std::min(channel, static_cast<uint32_t>(std::numeric_limits<uint16_t>::max()));
      *out_channel = static_cast<uint16_t>(channel);

      // Isaac Sim → Autoware mapping
      *out_return  = *in_return;   // echo_id → return_type
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<IsaacSimPointCloudAdapterNode>());
  rclcpp::shutdown();
  return 0;
}

