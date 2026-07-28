#include <array>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomCovarianceFixer : public rclcpp::Node
{
public:
  OdomCovarianceFixer() : Node("odom_covariance_fixer")
  {
    declare_parameter<std::string>("input_topic", "/utlidar/robot_odom");
    declare_parameter<std::string>("output_topic", "/utlidar/robot_odom_fixed");

    declare_parameter<std::vector<double>>(
      "pose_diagonal",
      std::vector<double>{0.05, 0.05, 0.10, 0.5, 0.5, 0.5});

    declare_parameter<std::vector<double>>(
      "twist_diagonal",
      std::vector<double>{0.05, 0.05, 0.10, 0.2, 0.2, 0.2});

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    const auto pose_diag = get_parameter("pose_diagonal").as_double_array();
    const auto twist_diag = get_parameter("twist_diagonal").as_double_array();

    if (pose_diag.size() != 6 || twist_diag.size() != 6) {
      throw std::runtime_error("pose_diagonal and twist_diagonal must each have 6 elements");
    }

    for (size_t i = 0; i < 6; ++i) {
      pose_diag_[i] = pose_diag[i];
      twist_diag_[i] = twist_diag[i];
    }

    pub_ = create_publisher<nav_msgs::msg::Odometry>(output_topic, rclcpp::SensorDataQoS());

    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&OdomCovarianceFixer::callback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Relaying %s -> %s", input_topic.c_str(), output_topic.c_str());
  }

private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out = *msg;

    out.pose.covariance.fill(0.0);
    out.twist.covariance.fill(0.0);

    for (size_t i = 0; i < 6; ++i) {
      out.pose.covariance[i * 6 + i] = pose_diag_[i];
      out.twist.covariance[i * 6 + i] = twist_diag_[i];
    }

    pub_->publish(out);
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  std::array<double, 6> pose_diag_{};
  std::array<double, 6> twist_diag_{};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomCovarianceFixer>());
  rclcpp::shutdown();
  return 0;
}