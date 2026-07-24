#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomCovarianceFixer : public rclcpp::Node
{
public:
  OdomCovarianceFixer() : Node("odom_covariance_fixer")
  {
    pub_ = create_publisher<nav_msgs::msg::Odometry>("/utlidar/robot_odom_fixed", rclcpp::SensorDataQoS());
    sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/utlidar/robot_odom", rclcpp::SensorDataQoS(),
      [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
        auto out = *msg;
        for (int i = 0; i < 36; i += 7) out.pose.covariance[i] = 0.01;
        for (int i = 0; i < 36; i += 7) out.twist.covariance[i] = 0.01;
        pub_->publish(out);
      });
  }
private:
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomCovarianceFixer>());
  rclcpp::shutdown();
  return 0;
}