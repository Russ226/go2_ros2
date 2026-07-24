#include <array>
#include <memory>
#include <string>
#include <cstring>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomConditioner : public rclcpp::Node
{
public:
    OdomConditioner()
    : Node("odom_conditioner"), last_stamp_ns_(0), have_last_stamp_(false)
    {
        input_topic_ = this->declare_parameter<std::string>(
            "input_topic", "/utlidar/robot_odom_raw");

        output_topic_ = this->declare_parameter<std::string>(
            "output_topic", "/utlidar/robot_odom");

        frame_id_ = this->declare_parameter<std::string>(
            "frame_id", "odom");

        child_frame_id_ = this->declare_parameter<std::string>(
            "child_frame_id", "base_link");

        drop_non_monotonic_timestamps_ = this->declare_parameter<bool>(
            "drop_non_monotonic_timestamps", true);

        pose_cov_diag_ = this->declare_parameter<std::vector<double>>(
            "pose_cov_diag", std::vector<double>{0.05, 0.05, 0.50, 0.30, 0.30, 0.08});

        twist_cov_diag_ = this->declare_parameter<std::vector<double>>(
            "twist_cov_diag", std::vector<double>{0.10, 0.10, 0.50, 0.30, 0.30, 0.10});

        if (pose_cov_diag_.size() != 6 || twist_cov_diag_.size() != 6)
        {
            throw std::runtime_error(
                "pose_cov_diag and twist_cov_diag must each contain exactly 6 values: "
                "[x, y, z, roll, pitch, yaw] and [vx, vy, vz, wx, wy, wz]");
        }

        auto qos = rclcpp::SensorDataQoS();

        pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
            output_topic_, qos);

        sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            input_topic_,
            qos,
            std::bind(&OdomConditioner::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(
            this->get_logger(),
            "Conditioning odometry: '%s' -> '%s'",
            input_topic_.c_str(),
            output_topic_.c_str());

        RCLCPP_INFO(
            this->get_logger(),
            "Pose covariance diag: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            pose_cov_diag_[0], pose_cov_diag_[1], pose_cov_diag_[2],
            pose_cov_diag_[3], pose_cov_diag_[4], pose_cov_diag_[5]);

        RCLCPP_INFO(
            this->get_logger(),
            "Twist covariance diag: [%.4f, %.4f, %.4f, %.4f, %.4f, %.4f]",
            twist_cov_diag_[0], twist_cov_diag_[1], twist_cov_diag_[2],
            twist_cov_diag_[3], twist_cov_diag_[4], twist_cov_diag_[5]);
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const int64_t stamp_ns =
            static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
            static_cast<int64_t>(msg->header.stamp.nanosec);

        if (drop_non_monotonic_timestamps_ && have_last_stamp_ && stamp_ns <= last_stamp_ns_)
        {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                2000,
                "Dropping odometry message with non-monotonic timestamp");
            return;
        }

        last_stamp_ns_ = stamp_ns;
        have_last_stamp_ = true;

        nav_msgs::msg::Odometry out = *msg;
        out.header.frame_id = frame_id_;
        out.child_frame_id = child_frame_id_;

        std::array<double, 36> pose_cov{};
        std::array<double, 36> twist_cov{};
        pose_cov.fill(0.0);
        twist_cov.fill(0.0);

        // Pose covariance diagonal: x, y, z, roll, pitch, yaw
        pose_cov[0]  = pose_cov_diag_[0];
        pose_cov[7]  = pose_cov_diag_[1];
        pose_cov[14] = pose_cov_diag_[2];
        pose_cov[21] = pose_cov_diag_[3];
        pose_cov[28] = pose_cov_diag_[4];
        pose_cov[35] = pose_cov_diag_[5];

        // Twist covariance diagonal: vx, vy, vz, wx, wy, wz
        twist_cov[0]  = twist_cov_diag_[0];
        twist_cov[7]  = twist_cov_diag_[1];
        twist_cov[14] = twist_cov_diag_[2];
        twist_cov[21] = twist_cov_diag_[3];
        twist_cov[28] = twist_cov_diag_[4];
        twist_cov[35] = twist_cov_diag_[5];

        std::memcpy(out.pose.covariance.data(), pose_cov.data(), 36 * sizeof(double));
        std::memcpy(out.twist.covariance.data(), twist_cov.data(), 36 * sizeof(double));

        pub_->publish(out);
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    std::string child_frame_id_;
    bool drop_non_monotonic_timestamps_;

    std::vector<double> pose_cov_diag_;
    std::vector<double> twist_cov_diag_;

    int64_t last_stamp_ns_;
    bool have_last_stamp_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomConditioner>());
    rclcpp::shutdown();
    return 0;
}