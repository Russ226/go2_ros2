#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <vector>

class ImuBiasCorrector : public rclcpp::Node
{
public:
    ImuBiasCorrector()
    : Node("imu_bias_corrector"),
      calibration_samples_(declare_parameter<int>("calibration_samples", 100)),
      calibrated_(false)
    {
        pub_ = create_publisher<sensor_msgs::msg::Imu>("/utlidar/imu_fixed", 10);
        sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/utlidar/imu", 10,
            std::bind(&ImuBiasCorrector::onImu, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
            "Calibrating IMU bias from first %d samples...", calibration_samples_);
    }

private:
    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        if (!calibrated_)
        {
            buffer_.push_back(*msg);
            if (static_cast<int>(buffer_.size()) >= calibration_samples_)
            {
                computeBias();
            }
            return;
        }

        auto out = *msg;
        out.header.stamp = now();
        out.header.frame_id = msg->header.frame_id;

        out.linear_acceleration.x = msg->linear_acceleration.x - bias_x_;
        out.linear_acceleration.y = msg->linear_acceleration.y - bias_y_;
        out.linear_acceleration.z = msg->linear_acceleration.z - bias_z_;

        normalize(out);
        pub_->publish(out);
    }

    void normalize(sensor_msgs::msg::Imu& out)
    {
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
    }

    void computeBias()
    {
        double sx = 0.0, sy = 0.0, sz = 0.0;
        for (const auto& m : buffer_)
        {
            sx += m.linear_acceleration.x;
            sy += m.linear_acceleration.y;
            sz += m.linear_acceleration.z;
        }
        const double n = static_cast<double>(buffer_.size());
        const double mx = sx / n;
        const double my = sy / n;
        const double mz = sz / n;

        bias_x_ = mx;
        bias_y_ = my;
        bias_z_ = mz - 9.80665;

        calibrated_ = true;
        buffer_.clear();
        buffer_.shrink_to_fit();

        RCLCPP_INFO(get_logger(),
            "IMU bias calibrated: bias=(%.4f, %.4f, %.4f) | "
            "corrected gravity will be ~9.81 m/s^2",
            bias_x_, bias_y_, bias_z_);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;

    int calibration_samples_;
    bool calibrated_;
    std::vector<sensor_msgs::msg::Imu> buffer_;

    double bias_x_ = 0.0;
    double bias_y_ = 0.0;
    double bias_z_ = 0.0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuBiasCorrector>());
    rclcpp::shutdown();
    return 0;
}