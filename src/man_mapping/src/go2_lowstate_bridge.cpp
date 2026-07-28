#include <array>
#include <algorithm>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"

#include "unitree_go/msg/low_state.hpp"

class LowStateBridge : public rclcpp::Node
{
public:
  using LowState = unitree_go::msg::LowState;

  LowStateBridge(): Node("go2_lowstate_bridge")
  {
    declare_parameter<std::string>("lowstate_topic", "/lf/lowstate");
    declare_parameter<std::string>("imu_topic", "/imu/data");
    declare_parameter<std::string>("joint_state_topic", "/joint_states");
    declare_parameter<std::string>("foot_contact_topic", "/foot_contacts");
    declare_parameter<std::string>("raw_foot_force_topic", "/foot_force/raw");
    declare_parameter<std::string>("base_frame", "base_link");

    declare_parameter<int>("num_active_joints", 12);
    declare_parameter<bool>("use_mode_filter", true);
    declare_parameter<bool>("publish_raw_foot_force", false);
    declare_parameter<double>("foot_force_threshold", 80.0);

    declare_parameter<std::vector<double>>(
        "imu.orientation_covariance",
        std::vector<double>{0.01, 0.0, 0.0,
                            0.0, 0.01, 0.0,
                            0.0, 0.0, 0.02});

    declare_parameter<std::vector<double>>(
        "imu.angular_velocity_covariance",
        std::vector<double>{0.001, 0.0, 0.0,
                            0.0, 0.001, 0.0,
                            0.0, 0.0, 0.002});

    declare_parameter<std::vector<double>>(
        "imu.linear_acceleration_covariance",
        std::vector<double>{0.1, 0.0, 0.0,
                            0.0, 0.1, 0.0,
                            0.0, 0.0, 0.2});

    lowstate_topic_ = get_parameter("lowstate_topic").as_string();
    imu_topic_ = get_parameter("imu_topic").as_string();
    joint_state_topic_ = get_parameter("joint_state_topic").as_string();
    foot_contact_topic_ = get_parameter("foot_contact_topic").as_string();
    raw_foot_force_topic_ = get_parameter("raw_foot_force_topic").as_string();
    base_frame_ = get_parameter("base_frame").as_string();

    num_active_joints_ = get_parameter("num_active_joints").as_int();
    use_mode_filter_ = get_parameter("use_mode_filter").as_bool();
    publish_raw_foot_force_ = get_parameter("publish_raw_foot_force").as_bool();
    foot_force_threshold_ = get_parameter("foot_force_threshold").as_double();

    const auto orientation_cov =
        get_parameter("imu.orientation_covariance").as_double_array();
    const auto angular_velocity_cov =
        get_parameter("imu.angular_velocity_covariance").as_double_array();
    const auto linear_acceleration_cov =
        get_parameter("imu.linear_acceleration_covariance").as_double_array();

    orientation_covariance_ = toCovarianceArray(
        orientation_cov, "imu.orientation_covariance");
    angular_velocity_covariance_ = toCovarianceArray(
        angular_velocity_cov, "imu.angular_velocity_covariance");
    linear_acceleration_covariance_ = toCovarianceArray(
        linear_acceleration_cov, "imu.linear_acceleration_covariance");

    if (num_active_joints_ != 12)
    {
      RCLCPP_WARN(
          get_logger(),
          "num_active_joints=%d, but Go2 legged joint mapping is expected to be 12. Continuing anyway.",
          num_active_joints_);
    }

    joint_names_ = {
        "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
        "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
        "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
        "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 50);
    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(joint_state_topic_, 50);
    foot_contact_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(foot_contact_topic_, 50);

    if (publish_raw_foot_force_)
    {
      foot_force_pub_ = create_publisher<std_msgs::msg::Int32MultiArray>(raw_foot_force_topic_, 50);
    }

    sub_ = create_subscription<LowState>(
        lowstate_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&LowStateBridge::lowStateCallback, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "LowStateBridge started");
    RCLCPP_INFO(get_logger(), "  lowstate_topic: %s", lowstate_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  imu_topic: %s", imu_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  joint_state_topic: %s", joint_state_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  foot_contact_topic: %s", foot_contact_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  raw_foot_force_topic: %s", raw_foot_force_topic_.c_str());
    RCLCPP_INFO(get_logger(), "  base_frame: %s", base_frame_.c_str());
    RCLCPP_INFO(get_logger(), "  num_active_joints: %d", num_active_joints_);
    RCLCPP_INFO(get_logger(), "  use_mode_filter: %s", use_mode_filter_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  publish_raw_foot_force: %s", publish_raw_foot_force_ ? "true" : "false");
    RCLCPP_INFO(get_logger(), "  foot_force_threshold: %.3f", foot_force_threshold_);
  }

private:
  static std::array<double, 9> toCovarianceArray(
      const std::vector<double> &values,
      const std::string &name)
  {
    if (values.size() != 9)
    {
      throw std::runtime_error(
          name + " must have exactly 9 elements, got " + std::to_string(values.size()));
    }

    std::array<double, 9> out{};
    std::copy(values.begin(), values.end(), out.begin());
    return out;
  }

  void lowStateCallback(const LowState::SharedPtr msg)
  {
    const rclcpp::Time stamp = now();

    publishImu(*msg, stamp);
    publishJointState(*msg, stamp);
    publishFootContacts(*msg);
  }

  void publishImu(const LowState &msg, const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = base_frame_;

    // Unitree IMU quaternion appears to be [w, x, y, z]
    imu.orientation.w = msg.imu_state.quaternion[0];
    imu.orientation.x = msg.imu_state.quaternion[1];
    imu.orientation.y = msg.imu_state.quaternion[2];
    imu.orientation.z = msg.imu_state.quaternion[3];

    imu.angular_velocity.x = msg.imu_state.gyroscope[0];
    imu.angular_velocity.y = msg.imu_state.gyroscope[1];
    imu.angular_velocity.z = msg.imu_state.gyroscope[2];

    imu.linear_acceleration.x = msg.imu_state.accelerometer[0];
    imu.linear_acceleration.y = msg.imu_state.accelerometer[1];
    imu.linear_acceleration.z = msg.imu_state.accelerometer[2];

    imu.orientation_covariance = orientation_covariance_;
    imu.angular_velocity_covariance = angular_velocity_covariance_;
    imu.linear_acceleration_covariance = linear_acceleration_covariance_;

    imu_pub_->publish(imu);
  }

  void publishJointState(const LowState &msg, const rclcpp::Time &stamp)
  {
    sensor_msgs::msg::JointState js;
    js.header.stamp = stamp;
    js.header.frame_id = base_frame_;

    const size_t available_motors = msg.motor_state.size();
    const size_t requested_joints = static_cast<size_t>(std::max(0, num_active_joints_));
    const size_t joint_count = std::min<size_t>(
        std::min<size_t>(requested_joints, joint_names_.size()),
        available_motors);

    if (joint_count < 12)
    {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Only %zu joints available for publishing, expected 12",
          joint_count);
    }

    js.name.reserve(joint_count);
    js.position.reserve(joint_count);
    js.velocity.reserve(joint_count);
    js.effort.reserve(joint_count);

    for (size_t i = 0; i < joint_count; ++i)
    {
      const auto &m = msg.motor_state[i];

      if (use_mode_filter_ && m.mode == 0)
      {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "motor_state[%zu] (%s) has mode=0, skipping",
            i, joint_names_[i].c_str());
        continue;
      }

      js.name.push_back(joint_names_[i]);
      js.position.push_back(m.q);
      js.velocity.push_back(m.dq);
      js.effort.push_back(m.tau_est);
    }

    joint_state_pub_->publish(js);
  }

  void publishFootContacts(const LowState &msg)
  {
    std_msgs::msg::Int32MultiArray contacts;
    contacts.data.resize(4, 0);

    for (size_t i = 0; i < 4 && i < msg.foot_force.size(); ++i)
    {
      contacts.data[i] = static_cast<double>(msg.foot_force[i]) > foot_force_threshold_ ? 1 : 0;
    }

    foot_contact_pub_->publish(contacts);

    if (publish_raw_foot_force_ && foot_force_pub_)
    {
      std_msgs::msg::Int32MultiArray raw_force;
      raw_force.data.resize(4, 0);

      for (size_t i = 0; i < 4 && i < msg.foot_force.size(); ++i)
      {
        raw_force.data[i] = static_cast<int32_t>(msg.foot_force[i]);
      }

      foot_contact_pub_->publish(contacts);
      foot_force_pub_->publish(raw_force);
    }
  }

  rclcpp::Subscription<LowState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr foot_contact_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr foot_force_pub_;

  std::string lowstate_topic_;
  std::string imu_topic_;
  std::string joint_state_topic_;
  std::string foot_contact_topic_;
  std::string raw_foot_force_topic_;
  std::string base_frame_;

  int num_active_joints_;
  bool use_mode_filter_;
  bool publish_raw_foot_force_;
  double foot_force_threshold_;

  std::array<double, 9> orientation_covariance_{};
  std::array<double, 9> angular_velocity_covariance_{};
  std::array<double, 9> linear_acceleration_covariance_{};

  std::array<std::string, 12> joint_names_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  try
  {
    auto node = std::make_shared<LowStateBridge>();
    rclcpp::spin(node);
  }
  catch (const std::exception &e)
  {
    fprintf(stderr, "Failed to start lowstate_bridge: %s\n", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}