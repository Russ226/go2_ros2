#include <cmath>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class PointCloudRotator : public rclcpp::Node
{
public:
    PointCloudRotator()
    : Node("pointcloud_rotator")
    {
        input_topic_ = declare_parameter<std::string>("input_topic", "/utlidar/cloud");
        output_topic_ = declare_parameter<std::string>("output_topic", "/utlidar/cloud_fixed");
        output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");
        roll_ = declare_parameter<double>("roll", 0.0);
        pitch_ = declare_parameter<double>("pitch", 0.0);
        yaw_ = declare_parameter<double>("yaw", 0.0);

        auto qos = rclcpp::SensorDataQoS();

        pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, qos);
        sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic_, qos,
            std::bind(&PointCloudRotator::callback, this, std::placeholders::_1));

        updateRotationMatrix();

        RCLCPP_INFO(
            get_logger(),
            "PointCloudRotator: %s -> %s  rpy=[%.6f, %.6f, %.6f] rad",
            input_topic_.c_str(), output_topic_.c_str(), roll_, pitch_, yaw_);
    }

private:
    void updateRotationMatrix()
    {
        const double cr = std::cos(roll_);
        const double sr = std::sin(roll_);
        const double cp = std::cos(pitch_);
        const double sp = std::sin(pitch_);
        const double cy = std::cos(yaw_);
        const double sy = std::sin(yaw_);

        // R = Rz(yaw) * Ry(pitch) * Rx(roll)
        r00_ = cy * cp;
        r01_ = cy * sp * sr - sy * cr;
        r02_ = cy * sp * cr + sy * sr;

        r10_ = sy * cp;
        r11_ = sy * sp * sr + cy * cr;
        r12_ = sy * sp * cr - cy * sr;

        r20_ = -sp;
        r21_ = cp * sr;
        r22_ = cp * cr;
    }

    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        auto out = *msg;

        if (!output_frame_id_.empty()) {
            out.header.frame_id = output_frame_id_;
        }

        try
        {
            sensor_msgs::PointCloud2Iterator<float> iter_x(out, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(out, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(out, "z");

            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                const float x = *iter_x;
                const float y = *iter_y;
                const float z = *iter_z;

                *iter_x = static_cast<float>(r00_ * x + r01_ * y + r02_ * z);
                *iter_y = static_cast<float>(r10_ * x + r11_ * y + r12_ * z);
                *iter_z = static_cast<float>(r20_ * x + r21_ * y + r22_ * z);
            }

            pub_->publish(out);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Failed rotating point cloud: %s", e.what());
        }
    }

    std::string input_topic_;
    std::string output_topic_;
    std::string output_frame_id_;

    double roll_, pitch_, yaw_;
    double r00_, r01_, r02_;
    double r10_, r11_, r12_;
    double r20_, r21_, r22_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudRotator>());
    rclcpp::shutdown();
    return 0;
}