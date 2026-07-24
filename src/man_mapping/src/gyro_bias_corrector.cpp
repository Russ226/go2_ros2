
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <array>
#include <optional>
#include <cmath>

class GyroBiasCorrector : public rclcpp::Node
{
public:
    GyroBiasCorrector() : Node("gyro_bias_corrector")
    {
        lin_thresh_ = declare_parameter<double>("lin_vel_threshold", 0.02);
        ang_thresh_ = declare_parameter<double>("ang_vel_threshold", 0.03);
        min_still_time_ = declare_parameter<double>("stationary_duration", 2.0);
        alpha_ = declare_parameter<double>("bias_alpha", 0.01);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom", rclcpp::QoS(50),
            std::bind(&GyroBiasCorrector::odomCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/utlidar/imu", rclcpp::QoS(200),
            std::bind(&GyroBiasCorrector::imuCallback, this, std::placeholders::_1));

        imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/utlidar/imu_corrected", rclcpp::QoS(200));
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &v = msg->twist.twist.linear;
        const auto &w = msg->twist.twist.angular;
        last_lin_speed_ = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
        last_ang_speed_ = std::sqrt(w.x * w.x + w.y * w.y + w.z * w.z);
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        const rclcpp::Time now = this->get_clock()->now();
        const bool is_still_now = (last_lin_speed_ < lin_thresh_) && (last_ang_speed_ < ang_thresh_);

        if (is_still_now)
        {
            if (!stationary_since_.has_value())
            {   
                stationary_since_ = now;
            }
            const double still_duration = (now - stationary_since_.value()).seconds();
            if (still_duration > min_still_time_)
            {
                const std::array<double, 3> gyro = {
                    msg->angular_velocity.x,
                    msg->angular_velocity.y,
                    msg->angular_velocity.z};
                for (size_t i = 0; i < 3; ++i)
                {
                    bias_[i] = (1.0 - alpha_) * bias_[i] + alpha_ * gyro[i];
                }
            }
        }
        else
        {
            stationary_since_.reset();
        }

        sensor_msgs::msg::Imu corrected = *msg;
        corrected.angular_velocity.x -= bias_[0];
        corrected.angular_velocity.y -= bias_[1];
        corrected.angular_velocity.z -= bias_[2];
        imu_pub_->publish(corrected);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    double lin_thresh_;
    double ang_thresh_;
    double min_still_time_;
    double alpha_;

    std::array<double, 3> bias_{0.0, 0.0, 0.0};
    std::optional<rclcpp::Time> stationary_since_;
    double last_lin_speed_{0.0};
    double last_ang_speed_{0.0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GyroBiasCorrector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}