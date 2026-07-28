#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class VirtualOdomPublisher : public rclcpp::Node
{
public:
    VirtualOdomPublisher()
        : Node("virtual_odom_publisher")
    {
        broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.w = 1.0;
        broadcaster_->sendTransform(tf);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/utlidar/robot_odom",
            rclcpp::QoS(rclcpp::KeepLast(20)).best_effort(),
            std::bind(&VirtualOdomPublisher::onWheelOdom, this, std::placeholders::_1));

        last_move_time_ = this->now();
    }

private:
    void onWheelOdom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        const auto &p = msg->pose.pose.position;
        const auto &q = msg->pose.pose.orientation;

        const double qn2 = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) ||
            !std::isfinite(q.z) || !std::isfinite(q.w) || qn2 < 1e-12)
        {
            return;
        }

        // Position-based zero-velocity detection
        const double dx = p.x - last_pose_.position.x;
        const double dy = p.y - last_pose_.position.y;
        const double dz = p.z - last_pose_.position.z;
        const double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist > POS_THRESH_)
        {
            // Moving: update last-move time, release freeze
            last_move_time_ = msg->header.stamp;
            frozen_ = false;
        }

        if (!frozen_)
        {
            // Cache current pose as the "frozen" reference
            frozen_pose_ = msg->pose.pose;
        }

        const double time_since_move = (this->now() - last_move_time_).seconds();
        if (time_since_move > TIME_THRESH_ && !frozen_)
        {
            // Just transitioned to stationary: lock the pose
            frozen_ = true;
            RCLCPP_INFO(get_logger(), "Stationary detected: freezing odom->base_link");
        }

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = msg->header.stamp;
        tf.header.frame_id = msg->header.frame_id; // odom
        tf.child_frame_id = msg->child_frame_id;   // base_link

        if (frozen_)
        {
            // Publish frozen pose
            tf.transform.translation.x = frozen_pose_.position.x;
            tf.transform.translation.y = frozen_pose_.position.y;
            tf.transform.translation.z = frozen_pose_.position.z;
            tf.transform.rotation = frozen_pose_.orientation;
        }
        else
        {
            // Moving: normalize and forward
            tf2::Quaternion q_tf(q.x, q.y, q.z, q.w);
            q_tf.normalize();

            tf.transform.translation.x = p.x;
            tf.transform.translation.y = p.y;
            tf.transform.translation.z = p.z;
            tf.transform.rotation = tf2::toMsg(q_tf);
        }

        last_pose_ = msg->pose.pose;
        broadcaster_->sendTransform(tf);
    }

    geometry_msgs::msg::Pose last_pose_;
    geometry_msgs::msg::Pose frozen_pose_;
    rclcpp::Time last_move_time_{0, 0, RCL_ROS_TIME};
    bool frozen_ = false;

    static constexpr double POS_THRESH_ = 0.02;  // 2 cm
    static constexpr double TIME_THRESH_ = 0.5;    // 0.5 seconds

    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualOdomPublisher>());
    rclcpp::shutdown();
    return 0;
}