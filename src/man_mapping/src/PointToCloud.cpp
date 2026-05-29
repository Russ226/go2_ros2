#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class CloudToScanNode : public rclcpp::Node
{
public:
    CloudToScanNode() : Node("cloud_to_scan_node")
    {
        declare_parameter("cloud_topic", "/utlidar/cloud");
        declare_parameter("scan_topic", "/scan");
        declare_parameter("output_frame", "");
        declare_parameter("min_height", -0.20);
        declare_parameter("max_height", 0.30);
        declare_parameter("angle_min", -M_PI);
        declare_parameter("angle_max", M_PI);
        declare_parameter("angle_increment", 0.005817764); // ~0.333 deg
        declare_parameter("scan_time", 0.1);
        declare_parameter("range_min", 0.10);
        declare_parameter("range_max", 30.0);
        declare_parameter("use_inf", true);

        cloud_topic_ = get_parameter("cloud_topic").as_string();
        scan_topic_ = get_parameter("scan_topic").as_string();
        output_frame_ = get_parameter("output_frame").as_string();
        min_height_ = get_parameter("min_height").as_double();
        max_height_ = get_parameter("max_height").as_double();
        angle_min_ = get_parameter("angle_min").as_double();
        angle_max_ = get_parameter("angle_max").as_double();
        angle_increment_ = get_parameter("angle_increment").as_double();
        scan_time_ = get_parameter("scan_time").as_double();
        range_min_ = get_parameter("range_min").as_double();
        range_max_ = get_parameter("range_max").as_double();
        use_inf_ = get_parameter("use_inf").as_bool();

        num_readings_ = static_cast<size_t>(
            std::ceil((angle_max_ - angle_min_) / angle_increment_));

        scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic_, 10);
        cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            cloud_topic_, rclcpp::SensorDataQoS(),
            std::bind(&CloudToScanNode::cloudCallback, this, std::placeholders::_1));
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        sensor_msgs::msg::LaserScan scan;
        scan.header = msg->header;
        if (!output_frame_.empty())
        {
            scan.header.frame_id = output_frame_;
        }

        scan.angle_min = angle_min_;
        scan.angle_max = angle_max_;
        scan.angle_increment = angle_increment_;
        scan.time_increment = 0.0;
        scan.scan_time = scan_time_;
        scan.range_min = range_min_;
        scan.range_max = range_max_;

        const float init = use_inf_
                               ? std::numeric_limits<float>::infinity()
                               : static_cast<float>(range_max_ + 1.0);

        scan.ranges.assign(num_readings_, init);

        sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            const float x = *iter_x;
            const float y = *iter_y;
            const float z = *iter_z;

            if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
            {
                continue;
            }

            if (z < min_height_ || z > max_height_)
            {
                continue;
            }

            const float range = std::sqrt(x * x + y * y);
            if (range < range_min_ || range > range_max_)
            {
                continue;
            }

            const float angle = std::atan2(y, x);
            if (angle < angle_min_ || angle > angle_max_)
            {
                continue;
            }

            const size_t index = static_cast<size_t>((angle - angle_min_) / angle_increment_);
            if (index >= scan.ranges.size())
            {
                continue;
            }

            if (range < scan.ranges[index])
            {
                scan.ranges[index] = range;
            }
        }

        scan_pub_->publish(scan);
    }

    std::string cloud_topic_;
    std::string scan_topic_;
    std::string output_frame_;

    double min_height_;
    double max_height_;
    double angle_min_;
    double angle_max_;
    double angle_increment_;
    double scan_time_;
    double range_min_;
    double range_max_;
    bool use_inf_;
    size_t num_readings_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CloudToScanNode>());
    rclcpp::shutdown();
    return 0;
}