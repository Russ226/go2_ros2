#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

class CloudFrameAccumulator : public rclcpp::Node
{
public:
  CloudFrameAccumulator() : Node("cloud_frame_accumulator")
  {
    declare_parameter<std::string>("input_topic", "/utlidar/cloud");
    declare_parameter<std::string>("output_topic", "/utlidar/cloud_accumulated");
    declare_parameter<int>("frames_per_cloud", 3);

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();
    frames_per_cloud_ = get_parameter("frames_per_cloud").as_int();

    if (frames_per_cloud_ < 1) {
      throw std::runtime_error("frames_per_cloud must be >= 1");
    }

    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_topic,
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic,
      rclcpp::QoS(rclcpp::KeepLast(10)).reliable(),
      std::bind(&CloudFrameAccumulator::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Accumulating %d frames: %s -> %s",
      frames_per_cloud_, input_topic.c_str(), output_topic.c_str());
  }

private:
  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    if (!have_layout_) {
      frame_id_ = msg->header.frame_id;
      point_step_ = msg->point_step;
      fields_ = msg->fields;
      is_bigendian_ = msg->is_bigendian;
      is_dense_ = msg->is_dense;
      have_layout_ = true;
    }

    const bool compatible =
      msg->header.frame_id == frame_id_ &&
      msg->point_step == point_step_ &&
      msg->fields.size() == fields_.size();

    if (!compatible) {
      RCLCPP_WARN(get_logger(), "Skipping cloud with incompatible layout");
      reset();
      return;
    }

    buffers_.insert(buffers_.end(), msg->data.begin(), msg->data.end());
    point_count_ += static_cast<size_t>(msg->width) * msg->height;
    ++frames_collected_;
    last_stamp_ = msg->header.stamp;

    if (frames_collected_ < frames_per_cloud_) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out;
    out.header.stamp = last_stamp_;
    out.header.frame_id = frame_id_;
    out.height = 1;
    out.width = static_cast<uint32_t>(point_count_);
    out.fields = fields_;
    out.is_bigendian = is_bigendian_;
    out.point_step = point_step_;
    out.row_step = out.point_step * out.width;
    out.is_dense = is_dense_;
    out.data = std::move(buffers_);

    pub_->publish(out);
    reset();
  }

  void reset()
  {
    buffers_.clear();
    point_count_ = 0;
    frames_collected_ = 0;
  }

  int frames_per_cloud_{3};
  bool have_layout_{false};
  bool is_bigendian_{false};
  bool is_dense_{true};
  uint32_t point_step_{0};
  size_t point_count_{0};
  int frames_collected_{0};

  std::string frame_id_;
  std::vector<sensor_msgs::msg::PointField> fields_;
  std::vector<uint8_t> buffers_;
  builtin_interfaces::msg::Time last_stamp_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudFrameAccumulator>());
  rclcpp::shutdown();
  return 0;
}