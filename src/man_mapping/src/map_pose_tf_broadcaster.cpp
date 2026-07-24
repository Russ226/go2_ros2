#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class MapPoseTfBroadcaster : public rclcpp::Node
{
public:
  MapPoseTfBroadcaster()
  : Node("map_pose_tf_broadcaster")
  {
    pose_topic_ = this->declare_parameter<std::string>("pose_topic", "/lidar_odometry/pose");
    map_frame_  = this->declare_parameter<std::string>("map_frame", "map");
    base_frame_ = this->declare_parameter<std::string>("base_frame", "base_link");
    publish_tf_ = this->declare_parameter<bool>("publish_tf", true);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      pose_topic_, rclcpp::SensorDataQoS(),
      std::bind(&MapPoseTfBroadcaster::odomCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(),
      "map_pose_tf_broadcaster started: %s -> [%s -> %s]",
      pose_topic_.c_str(), map_frame_.c_str(), base_frame_.c_str());
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (!publish_tf_) {
      return;
    }

    geometry_msgs::msg::TransformStamped t;

    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      t.header.stamp = this->now();
    } else {
      t.header.stamp = msg->header.stamp;
    }

    t.header.frame_id = map_frame_;
    t.child_frame_id  = base_frame_;

    t.transform.translation.x = msg->pose.pose.position.x;
    t.transform.translation.y = msg->pose.pose.position.y;
    t.transform.translation.z = msg->pose.pose.position.z;
    t.transform.rotation      = msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(t);
  }

  std::string pose_topic_;
  std::string map_frame_;
  std::string base_frame_;
  bool publish_tf_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapPoseTfBroadcaster>());
  rclcpp::shutdown();
  return 0;
}