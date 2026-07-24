#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class MolaPoseTfBroadcaster : public rclcpp::Node
{
public:
    MolaPoseTfBroadcaster()
        : Node("mola_pose_tf_broadcaster")
    {
        input_topic_ = declare_parameter<std::string>(
            "input_topic", "/lidar_odometry/pose");

        parent_frame_override_ = declare_parameter<std::string>(
            "parent_frame", "");

        child_frame_override_ = declare_parameter<std::string>(
            "child_frame", "");

        tf_broadcaster_ =
            std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            input_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(
                &MolaPoseTfBroadcaster::callback,
                this,
                std::placeholders::_1));

        RCLCPP_INFO(
            get_logger(),
            "Publishing TF from '%s'",
            input_topic_.c_str());
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &q = msg->pose.pose.orientation;

        const double norm_sq =
            q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;

        if (!std::isfinite(norm_sq) || norm_sq < 1e-12)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping pose with invalid quaternion");
            return;
        }

        geometry_msgs::msg::TransformStamped tf_msg;

        // Preserve the MOLA pose timestamp for TF time alignment.
        tf_msg.header.stamp = msg->header.stamp;

        tf_msg.header.frame_id =
            parent_frame_override_.empty()
                ? msg->header.frame_id
                : parent_frame_override_;

        tf_msg.child_frame_id =
            child_frame_override_.empty()
                ? msg->child_frame_id
                : child_frame_override_;

        if (tf_msg.header.frame_id.empty() ||
            tf_msg.child_frame_id.empty())
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Dropping pose with empty parent or child frame");
            return;
        }

        tf_msg.transform.translation.x = msg->pose.pose.position.x;
        tf_msg.transform.translation.y = msg->pose.pose.position.y;
        tf_msg.transform.translation.z = msg->pose.pose.position.z;

        const double inv_norm = 1.0 / std::sqrt(norm_sq);
        tf_msg.transform.rotation.x = q.x * inv_norm;
        tf_msg.transform.rotation.y = q.y * inv_norm;
        tf_msg.transform.rotation.z = q.z * inv_norm;
        tf_msg.transform.rotation.w = q.w * inv_norm;

        tf_broadcaster_->sendTransform(tf_msg);
    }

    std::string input_topic_;
    std::string parent_frame_override_;
    std::string child_frame_override_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MolaPoseTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}