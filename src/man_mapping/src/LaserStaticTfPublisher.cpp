#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"

class LaserStaticTfPublisher : public rclcpp::Node
{
public:
    LaserStaticTfPublisher() : Node("laser_static_tf_publisher")
    {
        declare_parameter("parent_frame", "base_link");
        declare_parameter("child_frame", "laser_frame");

        declare_parameter("x", 0.0);
        declare_parameter("y", 0.0);
        declare_parameter("z", 0.12);

        declare_parameter("roll", 0.0);
        declare_parameter("pitch", 0.0);
        declare_parameter("yaw", 0.0);

        broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = now();
        t.header.frame_id = get_parameter("parent_frame").as_string();
        t.child_frame_id = get_parameter("child_frame").as_string();

        t.transform.translation.x = get_parameter("x").as_double();
        t.transform.translation.y = get_parameter("y").as_double();
        t.transform.translation.z = get_parameter("z").as_double();

        tf2::Quaternion q;
        q.setRPY(
            get_parameter("roll").as_double(),
            get_parameter("pitch").as_double(),
            get_parameter("yaw").as_double());

        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        broadcaster_->sendTransform(t);
    }

private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserStaticTfPublisher>());
    rclcpp::shutdown();
    return 0;
}