#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuCovarianceFixer : public rclcpp::Node
{
public:
  ImuCovarianceFixer() : Node("imu_covariance_fixer")
  {
    declare_parameter<std::string>("input_topic", "/utlidar/imu");
    declare_parameter<std::string>("output_topic", "/utlidar/imu_fixed");

    declare_parameter<std::vector<double>>(
      "orientation_diagonal",
      std::vector<double>{0.05, 0.05, 0.20});

    declare_parameter<std::vector<double>>(
      "angular_velocity_diagonal",
      std::vector<double>{0.0025, 0.0025, 0.0025});

    declare_parameter<std::vector<double>>(
      "linear_acceleration_diagonal",
      std::vector<double>{0.25, 0.25, 0.25});

    const auto input_topic = get_parameter("input_topic").as_string();
    const auto output_topic = get_parameter("output_topic").as_string();

    const auto orientation_diag =
      get_parameter("orientation_diagonal").as_double_array();

    const auto angular_velocity_diag =
      get_parameter("angular_velocity_diagonal").as_double_array();

    const auto linear_acceleration_diag =
      get_parameter("linear_acceleration_diagonal").as_double_array();

    validate_diagonal("orientation_diagonal", orientation_diag);
    validate_diagonal("angular_velocity_diagonal", angular_velocity_diag);
    validate_diagonal("linear_acceleration_diagonal", linear_acceleration_diag);

    copy_diagonal(orientation_diag, orientation_diag_);
    copy_diagonal(angular_velocity_diag, angular_velocity_diag_);
    copy_diagonal(linear_acceleration_diag, linear_acceleration_diag_);

    pub_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic, rclcpp::QoS(rclcpp::KeepLast(20)).reliable());

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuCovarianceFixer::callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "Relaying IMU with covariances: %s -> %s",
      input_topic.c_str(), output_topic.c_str());
  }

private:
  static void validate_diagonal(const std::string& parameter_name, const std::vector<double>& diagonal)
  {
    if (diagonal.size() != 3) {
      throw std::runtime_error(
        parameter_name + " must contain exactly 3 variance values");
    }

    for (const double value : diagonal) {
      if (value <= 0.0) {
        throw std::runtime_error(
          parameter_name + " values must be strictly positive variances");
      }
    }
  }

  static void copy_diagonal(const std::vector<double>& input, std::array<double, 3>& output)
  {
    for (size_t i = 0; i < 3; ++i) {
      output[i] = input[i];
    }
  }

  static void set_covariance(std::array<double, 9>& covariance, const std::array<double, 3>& diagonal)
  {
    covariance.fill(0.0);
    covariance[0] = diagonal[0];
    covariance[4] = diagonal[1];
    covariance[8] = diagonal[2];
  }

  void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    sensor_msgs::msg::Imu out = *msg;

    set_covariance(out.orientation_covariance, orientation_diag_);
    set_covariance(out.angular_velocity_covariance, angular_velocity_diag_);
    set_covariance(out.linear_acceleration_covariance, linear_acceleration_diag_);

    pub_->publish(out);
  }

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

  std::array<double, 3> orientation_diag_{};
  std::array<double, 3> angular_velocity_diag_{};
  std::array<double, 3> linear_acceleration_diag_{};
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuCovarianceFixer>());
  rclcpp::shutdown();
  return 0;
}