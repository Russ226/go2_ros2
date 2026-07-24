#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

class ImuQuaternionNormalizer : public rclcpp::Node
{
public:
    ImuQuaternionNormalizer()
    : Node("imu_quaternion_normalizer")
    {
        input_topic_ = this->declare_parameter<std::string>(
            "input_topic", "/utlidar/imu");

        output_topic_ = this->declare_parameter<std::string>(
            "output_topic", "/utlidar/imu_fixed");

        drop_invalid_ = this->declare_parameter<bool>(
            "drop_invalid", true);

        auto qos = rclcpp::SensorDataQoS();

        pub_ = this->create_publisher<sensor_msgs::msg::Imu>(
            output_topic_, qos);

        sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            input_topic_,
            qos,
            std::bind(
                &ImuQuaternionNormalizer::imuCallback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Relaying IMU: '%s' -> '%s'",
            input_topic_.c_str(),
            output_topic_.c_str());
    }

private:
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto out = *msg;

        const double x = out.orientation.x;
        const double y = out.orientation.y;
        const double z = out.orientation.z;
        const double w = out.orientation.w;

        const double norm_sq = x * x + y * y + z * z + w * w;

        if (!std::isfinite(norm_sq) || norm_sq < 1e-12)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Dropping IMU message: invalid orientation quaternion "
                "(norm_sq=%.12f)",
                norm_sq);

            if (drop_invalid_)
                return;

            // Fallback identity orientation if explicitly configured not to drop.
            out.orientation.x = 0.0;
            out.orientation.y = 0.0;
            out.orientation.z = 0.0;
            out.orientation.w = 1.0;
        }
        else
        {
            const double inv_norm = 1.0 / std::sqrt(norm_sq);

            out.orientation.x = x * inv_norm;
            out.orientation.y = y * inv_norm;
            out.orientation.z = z * inv_norm;
            out.orientation.w = w * inv_norm;

            if (std::abs(norm_sq - 1.0) > 1e-6)
            {
                RCLCPP_WARN_THROTTLE(
                    get_logger(),
                    *get_clock(),
                    2000,
                    "Normalized IMU quaternion: norm_sq %.12f -> 1.0",
                    norm_sq);
            }
        }

        pub_->publish(out);
    }

    std::string input_topic_;
    std::string output_topic_;
    bool drop_invalid_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuQuaternionNormalizer>());
    rclcpp::shutdown();
    return 0;
}