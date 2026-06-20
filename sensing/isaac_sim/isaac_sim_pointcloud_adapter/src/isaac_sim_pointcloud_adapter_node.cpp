#include <algorithm>
#include <cstdint>
#include <cstring>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace
{

int getFieldOffset(const sensor_msgs::msg::PointCloud2 & cloud, const std::string & name)
{
  for (const auto & field : cloud.fields) {
    if (field.name == name) {
      return static_cast<int>(field.offset);
    }
  }
  return -1;
}

// Packed ROS rgb field (0x00RRGGBB) → luminance uint8
uint8_t rgbToIntensity(uint32_t packed_rgb)
{
  const uint8_t r = (packed_rgb >> 16) & 0xFF;
  const uint8_t g = (packed_rgb >> 8) & 0xFF;
  const uint8_t b = packed_rgb & 0xFF;
  return static_cast<uint8_t>(0.299f * r + 0.587f * g + 0.114f * b);
}

}  // namespace

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
    const int x_off          = getFieldOffset(*msg, "x");
    const int y_off          = getFieldOffset(*msg, "y");
    const int z_off          = getFieldOffset(*msg, "z");
    const int intensity_off  = getFieldOffset(*msg, "intensity");
    const int rgb_off        = getFieldOffset(*msg, "rgb");
    const int channel_id_off = getFieldOffset(*msg, "channel_id");
    const int echo_id_off    = getFieldOffset(*msg, "echo_id");

    if (x_off < 0 || y_off < 0 || z_off < 0) {
      RCLCPP_ERROR(get_logger(), "Input pointcloud missing x/y/z fields — dropping message");
      return;
    }

    if (!fields_logged_) {
      fields_logged_ = true;
      const char * intensity_src =
        intensity_off >= 0 ? "intensity (float)" :
        rgb_off >= 0       ? "rgb (luminance)" :
                             "mock 255";
      RCLCPP_INFO(
        get_logger(),
        "Input field layout (logged once):\n"
        "  intensity source: %s\n"
        "  channel_id: %s\n"
        "  echo_id:    %s",
        intensity_src,
        channel_id_off >= 0 ? "present" : "missing (mock 0)",
        echo_id_off >= 0    ? "present" : "missing (mock 0)");
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header = msg->header;
    out.height = msg->height;
    out.width = msg->width;
    out.is_dense = msg->is_dense;
    out.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(out);
    modifier.setPointCloud2Fields(
      6,
      "x", 1, sensor_msgs::msg::PointField::FLOAT32,
      "y", 1, sensor_msgs::msg::PointField::FLOAT32,
      "z", 1, sensor_msgs::msg::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::msg::PointField::UINT8,
      "return_type", 1, sensor_msgs::msg::PointField::UINT8,
      "channel", 1, sensor_msgs::msg::PointField::UINT16);

    const size_t n_points = static_cast<size_t>(msg->width) * msg->height;
    modifier.resize(n_points);

    sensor_msgs::PointCloud2Iterator<float>    out_x(out, "x");
    sensor_msgs::PointCloud2Iterator<float>    out_y(out, "y");
    sensor_msgs::PointCloud2Iterator<float>    out_z(out, "z");
    sensor_msgs::PointCloud2Iterator<uint8_t>  out_intensity(out, "intensity");
    sensor_msgs::PointCloud2Iterator<uint8_t>  out_return(out, "return_type");
    sensor_msgs::PointCloud2Iterator<uint16_t> out_channel(out, "channel");

    const uint8_t * in_data    = msg->data.data();
    const size_t    point_step = msg->point_step;

    for (size_t i = 0; i < n_points;
         ++i, ++out_x, ++out_y, ++out_z,
         ++out_intensity, ++out_return, ++out_channel)
    {
      const uint8_t * p = in_data + i * point_step;

      float x, y, z;
      std::memcpy(&x, p + x_off, sizeof(float));
      std::memcpy(&y, p + y_off, sizeof(float));
      std::memcpy(&z, p + z_off, sizeof(float));
      *out_x = x;
      *out_y = y;
      *out_z = z;

      // Intensity: float field → uint8; else RGB luminance; else max
      uint8_t intensity = 255;
      if (intensity_off >= 0) {
        float f;
        std::memcpy(&f, p + intensity_off, sizeof(float));
        intensity = static_cast<uint8_t>(std::clamp(f, 0.0f, 255.0f));
      } else if (rgb_off >= 0) {
        uint32_t packed;
        std::memcpy(&packed, p + rgb_off, sizeof(uint32_t));
        intensity = rgbToIntensity(packed);
      }
      *out_intensity = intensity;

      // Channel: channel_id (uint32) → uint16; else 0
      uint16_t channel = 0;
      if (channel_id_off >= 0) {
        uint32_t ch;
        std::memcpy(&ch, p + channel_id_off, sizeof(uint32_t));
        channel = static_cast<uint16_t>(
          std::min(ch, static_cast<uint32_t>(std::numeric_limits<uint16_t>::max())));
      }
      *out_channel = channel;

      // Return type: echo_id (uint8); else 0
      uint8_t return_type = 0;
      if (echo_id_off >= 0) {
        std::memcpy(&return_type, p + echo_id_off, sizeof(uint8_t));
      }
      *out_return = return_type;
    }

    pub_->publish(out);
  }

  std::string input_topic_;
  std::string output_topic_;
  bool fields_logged_{false};

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
