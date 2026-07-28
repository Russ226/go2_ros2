#include <cmath>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class OdomFixedPublisher : public rclcpp::Node
{
public:
    OdomFixedPublisher()
        : Node("odom_fixed_publisher")
    {
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/utlidar/robot_odom_fixed", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom",
            rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
            std::bind(&OdomFixedPublisher::onWheelOdom, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/utlidar/imu_fixed",
            rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
            std::bind(&OdomFixedPublisher::onImu, this, std::placeholders::_1));
    }

private:
    void onImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(imu_mutex_);
        latest_imu_ = *msg;
        imu_received_ = true;
    }

    void onWheelOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;

        const double qn2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w) || qn2 < 1e-12)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 1000,
                "Dropping invalid /utlidar/robot_odom pose");
            return;
        }

        const double lin = std::hypot(msg->twist.twist.linear.x,
                                      msg->twist.twist.linear.y);
        const double ang = std::abs(msg->twist.twist.angular.z);

        constexpr double LIN_THRESH = 0.03; // m/s
        constexpr double ANG_THRESH = 0.03; // rad/s

        nav_msgs::msg::Odometry out;
        out.header.stamp = msg->header.stamp;
        out.header.frame_id = msg->header.frame_id;     // odom
        out.child_frame_id = msg->child_frame_id;       // base_link

        if (lin < LIN_THRESH && ang < ANG_THRESH)
        {
            if (!last_valid_pose_initialized_)
            {
                last_valid_pose_ = msg->pose.pose;
                last_valid_pose_initialized_ = true;
            }

            out.pose.pose = last_valid_pose_;
            out.twist.twist.linear.x = 0.0;
            out.twist.twist.linear.y = 0.0;
            out.twist.twist.linear.z = 0.0;
            out.twist.twist.angular.x = 0.0;
            out.twist.twist.angular.y = 0.0;
            out.twist.twist.angular.z = 0.0;

            // High covariance when frozen to signal "we know we're stopped"
            for (size_t i = 0; i < 36; ++i) out.pose.covariance[i] = 0.0;
            for (size_t i = 0; i < 36; ++i) out.twist.covariance[i] = 0.0;
            out.pose.covariance[0]  = 1e-6;   // x
            out.pose.covariance[7]  = 1e-6;   // y
            out.pose.covariance[14] = 1e-6;   // z
            out.pose.covariance[21] = 1e-6;   // roll
            out.pose.covariance[28] = 1e-6;   // pitch
            out.pose.covariance[35] = 1e-6;   // yaw
            out.twist.covariance[0]  = 1e-6;   // vx
            out.twist.covariance[7]  = 1e-6;   // vy
            out.twist.covariance[14] = 1e-6;   // vz
            out.twist.covariance[21] = 1e-6;   // vroll
            out.twist.covariance[28] = 1e-6;   // vpitch
            out.twist.covariance[35] = 1e-6;   // vyaw
        }
        else
        {
            tf2::Quaternion q_odom(q.x, q.y, q.z, q.w);
            q_odom.normalize();

            tf2::Quaternion q_out = q_odom;

            {
                std::lock_guard<std::mutex> lock(imu_mutex_);
                if (imu_received_ && latest_imu_.orientation.w != 0.0)
                {
                    tf2::Quaternion q_imu(
                        latest_imu_.orientation.x,
                        latest_imu_.orientation.y,
                        latest_imu_.orientation.z,
                        latest_imu_.orientation.w);
                    q_imu.normalize();

                    double imu_roll, imu_pitch, imu_yaw;
                    tf2::Matrix3x3(q_imu).getRPY(imu_roll, imu_pitch, imu_yaw);

                    double odom_roll, odom_pitch, odom_yaw;
                    tf2::Matrix3x3(q_odom).getRPY(odom_roll, odom_pitch, odom_yaw);

                    q_out.setRPY(imu_roll, imu_pitch, odom_yaw);
                    q_out.normalize();
                }
            }

            out.pose.pose.position = p;
            out.pose.pose.orientation = tf2::toMsg(q_out);
            out.twist = msg->twist;

            // Moderate covariance when moving
            for (size_t i = 0; i < 36; ++i) out.pose.covariance[i] = 0.0;
            for (size_t i = 0; i < 36; ++i) out.twist.covariance[i] = 0.0;
            out.pose.covariance[0]  = 0.01;   // x
            out.pose.covariance[7]  = 0.01;   // y
            out.pose.covariance[14] = 0.01;   // z
            out.pose.covariance[21] = 0.01;   // roll
            out.pose.covariance[28] = 0.01;   // pitch
            out.pose.covariance[35] = 0.05;   // yaw (odom yaw is less certain)
            out.twist.covariance[0]  = 0.01;  // vx
            out.twist.covariance[7]  = 0.01;  // vy
            out.twist.covariance[14] = 0.01;  // vz
            out.twist.covariance[21] = 0.01;  // vroll
            out.twist.covariance[28] = 0.01;  // vpitch
            out.twist.covariance[35] = 0.05;  // vyaw

            last_valid_pose_ = out.pose.pose;
            last_valid_pose_initialized_ = true;
        }

        odom_pub_->publish(out);
    }

    geometry_msgs::msg::Pose last_valid_pose_;
    bool last_valid_pose_initialized_ = false;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    sensor_msgs::msg::Imu latest_imu_;
    bool imu_received_ = false;
    std::mutex imu_mutex_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomFixedPublisher>());
    rclcpp::shutdown();
    return 0;
}