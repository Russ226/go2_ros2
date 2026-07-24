#include <algorithm>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

class VirtualOdomPublisher : public rclcpp::Node
{
public:
    VirtualOdomPublisher()
        : Node("virtual_odom_publisher")
    {
        odom_topic_ = declare_parameter<std::string>(
            "odom_topic", "/utlidar/robot_odom_fixed");

        odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
        base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

        linear_stationary_threshold_ = declare_parameter<double>(
            "linear_stationary_threshold", 0.025);

        angular_stationary_threshold_ = declare_parameter<double>(
            "angular_stationary_threshold", 0.04);

        stationary_hold_sec_ = declare_parameter<double>(
            "stationary_hold_sec", 0.75);

        enable_stationary_freeze_ = declare_parameter<bool>(
            "enable_stationary_freeze", true);

        broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        const auto qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            qos,
            std::bind(
                &VirtualOdomPublisher::onOdom,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Listening on '%s', publishing %s -> %s, stationary freeze: %s",
            odom_topic_.c_str(),
            odom_frame_.c_str(),
            base_frame_.c_str(),
            enable_stationary_freeze_ ? "enabled" : "disabled");
    }

private:
    void onOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped measured_tf;
        measured_tf.header.stamp = msg->header.stamp;
        measured_tf.header.frame_id = odom_frame_;
        measured_tf.child_frame_id = base_frame_;

        measured_tf.transform.translation.x = msg->pose.pose.position.x;
        measured_tf.transform.translation.y = msg->pose.pose.position.y;
        measured_tf.transform.translation.z = msg->pose.pose.position.z;
        measured_tf.transform.rotation = msg->pose.pose.orientation;

        const double q_norm_sq =
            measured_tf.transform.rotation.x * measured_tf.transform.rotation.x +
            measured_tf.transform.rotation.y * measured_tf.transform.rotation.y +
            measured_tf.transform.rotation.z * measured_tf.transform.rotation.z +
            measured_tf.transform.rotation.w * measured_tf.transform.rotation.w;

        if (q_norm_sq < 1e-8)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Ignoring odometry with invalid zero quaternion");
            return;
        }

        const double inv_q_norm = 1.0 / std::sqrt(q_norm_sq);
        measured_tf.transform.rotation.x *= inv_q_norm;
        measured_tf.transform.rotation.y *= inv_q_norm;
        measured_tf.transform.rotation.z *= inv_q_norm;
        measured_tf.transform.rotation.w *= inv_q_norm;

        const auto &twist = msg->twist.twist;

        const double planar_speed = std::hypot(twist.linear.x, twist.linear.y);

        const double yaw_rate = std::abs(twist.angular.z);

        const bool reported_stationary =
            planar_speed < linear_stationary_threshold_ &&
            yaw_rate < angular_stationary_threshold_;

        const rclcpp::Time stamp(msg->header.stamp);

        if (!reported_stationary)
        {
            stationary_since_.reset();
            frozen_ = false;
            last_live_tf_ = measured_tf;
            have_last_live_tf_ = true;
            broadcaster_->sendTransform(measured_tf);
            return;
        }

        if (!have_last_live_tf_)
        {
            last_live_tf_ = measured_tf;
            have_last_live_tf_ = true;
        }

        if (!stationary_since_)
        {
            stationary_since_ = stamp;
        }

        const double stationary_duration =
            (stamp - *stationary_since_).seconds();

        if (enable_stationary_freeze_ &&
            stationary_duration >= stationary_hold_sec_)
        {
            if (!frozen_)
            {
                frozen_tf_ = last_live_tf_;
                frozen_ = true;

                RCLCPP_INFO(
                    get_logger(),
                    "Stationary for %.2f s; freezing odom -> base_link",
                    stationary_duration);
            }

            auto tf = frozen_tf_;
            tf.header.stamp = msg->header.stamp;
            broadcaster_->sendTransform(tf);
            return;
        }

        last_live_tf_ = measured_tf;
        broadcaster_->sendTransform(measured_tf);
    }

    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;

    double linear_stationary_threshold_{0.025};
    double angular_stationary_threshold_{0.04};
    double stationary_hold_sec_{0.75};
    bool enable_stationary_freeze_{true};

    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    bool have_last_live_tf_{false};
    bool frozen_{false};

    geometry_msgs::msg::TransformStamped last_live_tf_;
    geometry_msgs::msg::TransformStamped frozen_tf_;

    std::optional<rclcpp::Time> stationary_since_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}