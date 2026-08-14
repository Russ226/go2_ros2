#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuBiasCorrector : public rclcpp::Node
{
public:
  ImuBiasCorrector() : Node("imu_bias_corrector")
  {
    declare_parameter<std::string>("input_topic", "/utlidar/imu");
    declare_parameter<std::string>(
      "output_topic", "/utlidar/imu_bias_corrected");

    // Keep the Go2 stationary during this initial period.
    declare_parameter<double>("startup_calibration_seconds", 8.0);

    // Stationary test:
    declare_parameter<double>("gyro_stationary_threshold_rad_s", 0.08);
    declare_parameter<double>("accel_magnitude_tolerance_m_s2", 0.75);
    declare_parameter<double>("gravity_m_s2", 9.80665);

    // EMA factor applied only when stationary and calibrated.
    // 0 disables runtime bias adaptation.
    declare_parameter<double>("stationary_bias_alpha", 0.001);

    // Covariances written into the output message.
    declare_parameter<std::vector<double>>(
      "orientation_diagonal",
      std::vector<double>{0.05, 0.05, 0.20});

    declare_parameter<std::vector<double>>(
      "angular_velocity_diagonal",
      std::vector<double>{0.0025, 0.0025, 0.0025});

    declare_parameter<std::vector<double>>(
      "linear_acceleration_diagonal",
      std::vector<double>{0.25, 0.25, 0.25});

    input_topic_ = get_parameter("input_topic").as_string();
    output_topic_ = get_parameter("output_topic").as_string();

    startup_calibration_seconds_ =
      get_parameter("startup_calibration_seconds").as_double();

    gyro_stationary_threshold_rad_s_ =
      get_parameter("gyro_stationary_threshold_rad_s").as_double();

    accel_magnitude_tolerance_m_s2_ =
      get_parameter("accel_magnitude_tolerance_m_s2").as_double();

    gravity_m_s2_ = get_parameter("gravity_m_s2").as_double();

    stationary_bias_alpha_ =
      get_parameter("stationary_bias_alpha").as_double();

    load_diagonal(
      "orientation_diagonal",
      get_parameter("orientation_diagonal").as_double_array(),
      orientation_diagonal_);

    load_diagonal(
      "angular_velocity_diagonal",
      get_parameter("angular_velocity_diagonal").as_double_array(),
      angular_velocity_diagonal_);

    load_diagonal(
      "linear_acceleration_diagonal",
      get_parameter("linear_acceleration_diagonal").as_double_array(),
      linear_acceleration_diagonal_);

    if (startup_calibration_seconds_ <= 0.0) {
      throw std::runtime_error("startup_calibration_seconds must be > 0");
    }

    if (gyro_stationary_threshold_rad_s_ <= 0.0) {
      throw std::runtime_error(
        "gyro_stationary_threshold_rad_s must be > 0");
    }

    if (accel_magnitude_tolerance_m_s2_ <= 0.0) {
      throw std::runtime_error(
        "accel_magnitude_tolerance_m_s2 must be > 0");
    }

    if (stationary_bias_alpha_ < 0.0 || stationary_bias_alpha_ > 1.0) {
      throw std::runtime_error(
        "stationary_bias_alpha must be in [0, 1]");
    }

    pub_ = create_publisher<sensor_msgs::msg::Imu>(
      output_topic_,
      rclcpp::QoS(rclcpp::KeepLast(50)).reliable());

    sub_ = create_subscription<sensor_msgs::msg::Imu>(
      input_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&ImuBiasCorrector::callback, this, std::placeholders::_1));

    calibration_start_ = now();

    RCLCPP_INFO(
      get_logger(),
      "IMU bias corrector: %s -> %s. Keep robot still for %.1f seconds.",
      input_topic_.c_str(),
      output_topic_.c_str(),
      startup_calibration_seconds_);
  }

private:
  static double norm3(double x, double y, double z)
  {
    return std::sqrt(x * x + y * y + z * z);
  }

  static void load_diagonal(
    const std::string &name,
    const std::vector<double> &input,
    std::array<double, 3> &output)
  {
    if (input.size() != 3) {
      throw std::runtime_error(name + " must contain exactly 3 values");
    }

    for (size_t i = 0; i < 3; ++i) {
      if (input[i] <= 0.0) {
        throw std::runtime_error(
          name + " values must be strictly positive variances");
      }
      output[i] = input[i];
    }
  }

  static void set_covariance(
    std::array<double, 9> &covariance,
    const std::array<double, 3> &diagonal)
  {
    covariance.fill(0.0);
    covariance[0] = diagonal[0];
    covariance[4] = diagonal[1];
    covariance[8] = diagonal[2];
  }

  bool is_stationary(const sensor_msgs::msg::Imu &msg) const
  {
    const double gyro_norm = norm3(
      msg.angular_velocity.x,
      msg.angular_velocity.y,
      msg.angular_velocity.z);

    const double accel_norm = norm3(
      msg.linear_acceleration.x,
      msg.linear_acceleration.y,
      msg.linear_acceleration.z);

    return gyro_norm < gyro_stationary_threshold_rad_s_ &&
      std::abs(accel_norm - gravity_m_s2_) <
      accel_magnitude_tolerance_m_s2_;
  }

  void update_startup_bias(const sensor_msgs::msg::Imu &msg)
  {
    gyro_sum_[0] += msg.angular_velocity.x;
    gyro_sum_[1] += msg.angular_velocity.y;
    gyro_sum_[2] += msg.angular_velocity.z;
    ++startup_samples_;
  }

  void finalize_startup_calibration()
  {
    if (startup_samples_ == 0) {
      RCLCPP_WARN(
        get_logger(),
        "No IMU samples arrived during startup calibration; using zero bias.");
    } else {
      for (size_t i = 0; i < 3; ++i) {
        gyro_bias_[i] = gyro_sum_[i] /
          static_cast<double>(startup_samples_);
      }

      RCLCPP_INFO(
        get_logger(),
        "Gyro bias calibrated from %zu samples: [%.6f, %.6f, %.6f] rad/s",
        startup_samples_,
        gyro_bias_[0],
        gyro_bias_[1],
        gyro_bias_[2]);
    }

    calibrated_ = true;
  }

  void update_runtime_bias(const sensor_msgs::msg::Imu &msg)
  {
    if (stationary_bias_alpha_ == 0.0) {
      return;
    }

    gyro_bias_[0] =
      (1.0 - stationary_bias_alpha_) * gyro_bias_[0] +
      stationary_bias_alpha_ * msg.angular_velocity.x;

    gyro_bias_[1] =
      (1.0 - stationary_bias_alpha_) * gyro_bias_[1] +
      stationary_bias_alpha_ * msg.angular_velocity.y;

    gyro_bias_[2] =
      (1.0 - stationary_bias_alpha_) * gyro_bias_[2] +
      stationary_bias_alpha_ * msg.angular_velocity.z;
  }

  void callback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    const auto elapsed = now() - calibration_start_;

    if (!calibrated_) {
      // Initial calibration intentionally assumes robot is stationary.
      // Do not start walking until the node prints its calibrated bias.
      update_startup_bias(*msg);

      if (elapsed.seconds() >= startup_calibration_seconds_) {
        finalize_startup_calibration();
      }
    }

    sensor_msgs::msg::Imu out = *msg;

    out.angular_velocity.x -= gyro_bias_[0];
    out.angular_velocity.y -= gyro_bias_[1];
    out.angular_velocity.z -= gyro_bias_[2];

    // Use raw input to decide whether the body is stationary.
    if (calibrated_ && is_stationary(*msg)) {
      update_runtime_bias(*msg);
    }

    set_covariance(out.orientation_covariance, orientation_diagonal_);
    set_covariance(
      out.angular_velocity_covariance, angular_velocity_diagonal_);
    set_covariance(
      out.linear_acceleration_covariance, linear_acceleration_diagonal_);

    pub_->publish(out);

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "calibrated=%s bias=[%.5f %.5f %.5f] rad/s",
      calibrated_ ? "true" : "false",
      gyro_bias_[0],
      gyro_bias_[1],
      gyro_bias_[2]);
  }

  std::string input_topic_;
  std::string output_topic_;

  double startup_calibration_seconds_{8.0};
  double gyro_stationary_threshold_rad_s_{0.08};
  double accel_magnitude_tolerance_m_s2_{0.75};
  double gravity_m_s2_{9.80665};
  double stationary_bias_alpha_{0.001};

  bool calibrated_{false};
  size_t startup_samples_{0};

  rclcpp::Time calibration_start_;
  std::array<double, 3> gyro_sum_{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_bias_{0.0, 0.0, 0.0};

  std::array<double, 3> orientation_diagonal_{};
  std::array<double, 3> angular_velocity_diagonal_{};
  std::array<double, 3> linear_acceleration_diagonal_{};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuBiasCorrector>());
  rclcpp::shutdown();
  return 0;
}