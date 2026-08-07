#include <cstring>
#include <memory>
#include <stdexcept>
#include <string>

#include <gst/app/gstappsrc.h>
#include <gst/app/gstappsink.h>
#include <gst/video/video.h>
#include <gst/gst.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <unitree_go/msg/go2_front_video_data.hpp>

class Go2FrontVideoToImage final : public rclcpp::Node
{
public:
  Go2FrontVideoToImage()
      : Node("go2_video_to_image")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/frontvideostream");
    output_topic_ = declare_parameter<std::string>("output_topic", "/utlidar/camera");
    resolution_ = declare_parameter<std::string>("resolution", "720p");
    frame_id_ = declare_parameter<std::string>("frame_id", "front_camera_optical_frame");
    network_interface_ = declare_parameter<std::string>("network_interface", "enp3s0");
    rclcpp::QoS source_qos(rclcpp::KeepLast(1));
    source_qos.reliable();

    image_pub_ = create_publisher<sensor_msgs::msg::Image>(
        output_topic_, rclcpp::SensorDataQoS());
    video_sub_ = create_subscription<unitree_go::msg::Go2FrontVideoData>(
        input_topic_, source_qos,
        std::bind(&Go2FrontVideoToImage::on_video, this, std::placeholders::_1));
    gst_init(nullptr, nullptr);
    create_pipeline();

    RCLCPP_INFO(get_logger(), "Decoding %s from %s and publishing BGR8 Images on %s",
                resolution_.c_str(), input_topic_.c_str(), output_topic_.c_str());
  }

  ~Go2FrontVideoToImage() override
  {
    if (pipeline_ != nullptr)
    {
      gst_element_set_state(pipeline_, GST_STATE_NULL);
      gst_object_unref(pipeline_);
    }
  }

private:
  void create_pipeline()
{
  GError *error = nullptr;

  const std::string pipeline_text =
      "udpsrc address=230.1.1.1 port=1720 "
      "multicast-iface=" + network_interface_ + " "
      "auto-multicast=true "
      "! application/x-rtp,media=video,encoding-name=H264,payload=96 "
      "! rtph264depay "
      "! h264parse "
      "! avdec_h264 "
      "! videoconvert "
      "! video/x-raw,format=BGR "
      "! appsink name=ros_sink "
      "emit-signals=true sync=false max-buffers=1 drop=true";

  pipeline_ = gst_parse_launch(pipeline_text.c_str(), &error);

  if (pipeline_ == nullptr || error != nullptr) {
    const std::string message =
        error != nullptr ? error->message : "unknown GStreamer error";

    if (error != nullptr) {
      g_error_free(error);
    }

    throw std::runtime_error(
        "Unable to construct Go2 RTP camera pipeline: " + message);
  }

  appsink_ = GST_APP_SINK(
      gst_bin_get_by_name(GST_BIN(pipeline_), "ros_sink"));

  if (appsink_ == nullptr) {
    throw std::runtime_error(
        "Unable to find appsink 'ros_sink' in RTP camera pipeline");
  }

  g_signal_connect(
      appsink_,
      "new-sample",
      G_CALLBACK(&Go2FrontVideoToImage::new_sample),
      this);

  const GstStateChangeReturn state_result =
      gst_element_set_state(pipeline_, GST_STATE_PLAYING);

  if (state_result == GST_STATE_CHANGE_FAILURE) {
    throw std::runtime_error(
        "Unable to start Go2 RTP camera pipeline");
  }
}
  void check_gst_bus()
  {
    GstBus *bus = gst_element_get_bus(pipeline_);
    GstMessage *message = nullptr;

    while ((message = gst_bus_pop(bus)) != nullptr)
    {
      if (GST_MESSAGE_TYPE(message) == GST_MESSAGE_ERROR)
      {
        GError *error = nullptr;
        gchar *debug = nullptr;

        gst_message_parse_error(message, &error, &debug);

        RCLCPP_ERROR(
            get_logger(),
            "GStreamer error: %s | debug: %s",
            error != nullptr ? error->message : "unknown",
            debug != nullptr ? debug : "none");

        if (error != nullptr)
        {
          g_error_free(error);
        }
        if (debug != nullptr)
        {
          g_free(debug);
        }
      }

      gst_message_unref(message);
    }

    gst_object_unref(bus);
  }

  void on_video(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg)
  {
    const std::vector<uint8_t> *payload = nullptr;
    if (resolution_ == "720p")
      payload = &msg->video720p;
    else if (resolution_ == "360p")
      payload = &msg->video360p;
    else if (resolution_ == "180p")
      payload = &msg->video180p;
    else
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                            "Unknown resolution '%s'; use 720p, 360p, or 180p", resolution_.c_str());
      return;
    }

    if (payload->empty())
      return;

    GstBuffer *buffer = gst_buffer_new_allocate(nullptr, payload->size(), nullptr);
    GstMapInfo map;
    if (!gst_buffer_map(buffer, &map, GST_MAP_WRITE))
    {
      gst_buffer_unref(buffer);
      return;
    }
    std::memcpy(map.data, payload->data(), payload->size());
    gst_buffer_unmap(buffer, &map);

    const GstFlowReturn result = gst_app_src_push_buffer(appsrc_, buffer);
    if (result != GST_FLOW_OK)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "GStreamer rejected H.264 chunk; flow result: %d", result);
    }
    check_gst_bus();
    if (result != GST_FLOW_OK)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "GStreamer rejected an H.264 buffer (flow result %d)", result);
    }
  }

  static GstFlowReturn new_sample(GstAppSink *sink, gpointer user_data)
  {
    auto *self = static_cast<Go2FrontVideoToImage *>(user_data);
    GstSample *sample = gst_app_sink_pull_sample(sink);
    if (sample == nullptr)
      return GST_FLOW_ERROR;

    GstCaps *caps = gst_sample_get_caps(sample);
    GstBuffer *buffer = gst_sample_get_buffer(sample);
    GstVideoInfo info;
    if (caps == nullptr || buffer == nullptr || !gst_video_info_from_caps(&info, caps))
    {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    GstVideoFrame frame;
    if (!gst_video_frame_map(&frame, &info, buffer, GST_MAP_READ))
    {
      gst_sample_unref(sample);
      return GST_FLOW_ERROR;
    }

    auto image = sensor_msgs::msg::Image();
    image.header.stamp = self->now();
    image.header.frame_id = self->frame_id_;
    image.height = GST_VIDEO_INFO_HEIGHT(&info);
    image.width = GST_VIDEO_INFO_WIDTH(&info);
    image.encoding = "bgr8";
    image.is_bigendian = false;
    image.step = GST_VIDEO_FRAME_PLANE_STRIDE(&frame, 0);
    image.data.resize(static_cast<size_t>(image.step) * image.height);
    std::memcpy(image.data.data(), GST_VIDEO_FRAME_PLANE_DATA(&frame, 0), image.data.size());

    gst_video_frame_unmap(&frame);
    gst_sample_unref(sample);
    self->image_pub_->publish(std::move(image));
    return GST_FLOW_OK;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string resolution_;
  std::string frame_id_;
  std::string network_interface_;
  rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr video_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  GstElement *pipeline_{nullptr};
  GstAppSrc *appsrc_{nullptr};
  GstAppSink *appsink_{nullptr};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2FrontVideoToImage>());
  rclcpp::shutdown();
  return 0;
}