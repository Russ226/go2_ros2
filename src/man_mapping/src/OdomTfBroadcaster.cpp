#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

class OdomTfBroadcaster : public rclcpp::Node
{
public:
    OdomTfBroadcaster() : Node("odom_tf_broadcaster")
    {
        declare_parameter("odom_topic", "/utlidar/robot_odom");
        declare_parameter("odom_frame", "odom");
        declare_parameter("base_frame", "base_link");
        declare_parameter("base_footprint", "base_footprint");

        odom_topic_ = get_parameter("odom_topic").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        base_frame_ = get_parameter("base_frame").as_string();
        base_footprint_ = get_parameter("base_footprint").as_string();

        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        sub_ = create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, rclcpp::SensorDataQoS(),
            std::bind(&OdomTfBroadcaster::callback, this, std::placeholders::_1));
    }

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        tf2::Quaternion q_body;
        tf2::fromMsg(msg->pose.pose.orientation, q_body);

        double roll, pitch, yaw;
        tf2::Matrix3x3(q_body).getRPY(roll, pitch, yaw);

        geometry_msgs::msg::TransformStamped odom_to_footprint;
        odom_to_footprint.header.stamp = msg->header.stamp;
        odom_to_footprint.header.frame_id = odom_frame_;
        odom_to_footprint.child_frame_id = base_footprint_;
        odom_to_footprint.transform.translation.x = msg->pose.pose.position.x;
        odom_to_footprint.transform.translation.y = msg->pose.pose.position.y;
        odom_to_footprint.transform.translation.z = 0.0;

        tf2::Quaternion q_yaw;
        q_yaw.setRPY(0.0, 0.0, yaw);
        odom_to_footprint.transform.rotation = tf2::toMsg(q_yaw);

        geometry_msgs::msg::TransformStamped footprint_to_base;
        footprint_to_base.header.stamp = msg->header.stamp;
        footprint_to_base.header.frame_id = base_footprint_;
        footprint_to_base.child_frame_id = base_frame_;
        footprint_to_base.transform.translation.x = 0.0;
        footprint_to_base.transform.translation.y = 0.0;
        footprint_to_base.transform.translation.z = msg->pose.pose.position.z;

        tf2::Quaternion q_rp;
        q_rp.setRPY(roll, pitch, 0.0);
        footprint_to_base.transform.rotation = tf2::toMsg(q_rp);

        tf_broadcaster_->sendTransform(odom_to_footprint);
        tf_broadcaster_->sendTransform(footprint_to_base);
    }

    std::string odom_topic_;
    std::string odom_frame_;
    std::string base_frame_;
    std::string base_footprint_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomTfBroadcaster>());
    rclcpp::shutdown();
    return 0;
}