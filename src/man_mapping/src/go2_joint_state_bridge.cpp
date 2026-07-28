#include <array>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <unitree_go/msg/low_state.hpp>

class Go2JointStateBridge final : public rclcpp::Node
{
public:
  Go2JointStateBridge()
  : Node("go2_joint_state_bridge")
  {
    lowstate_topic_ = declare_parameter<std::string>(
      "lowstate_topic", "/lowstate");

    joint_states_topic_ = declare_parameter<std::string>(
      "joint_states_topic", "/joint_states");

    const auto default_joint_names = std::vector<std::string>{
      "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
      "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
      "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
      "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"
    };

    joint_names_ = declare_parameter<std::vector<std::string>>(
      "joint_names", default_joint_names);

    motor_indices_ = declare_parameter<std::vector<int64_t>>(
      "motor_indices", std::vector<int64_t>{
        0, 1, 2,
        3, 4, 5,
        6, 7, 8,
        9, 10, 11
      });

    signs_ = declare_parameter<std::vector<double>>(
      "signs", std::vector<double>(12, 1.0));

    offsets_ = declare_parameter<std::vector<double>>(
      "offsets", std::vector<double>(12, 0.0));

    validate_configuration();

    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    sensor_qos.best_effort();
    sensor_qos.durability_volatile();

    joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
      joint_states_topic_, sensor_qos);

    lowstate_sub_ = create_subscription<unitree_go::msg::LowState>(
      lowstate_topic_,
      sensor_qos,
      std::bind(
        &Go2JointStateBridge::on_lowstate,
        this,
        std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging '%s' -> '%s' for %zu joints",
      lowstate_topic_.c_str(),
      joint_states_topic_.c_str(),
      joint_names_.size());
  }

private:
  void validate_configuration()
  {
    constexpr size_t kGo2LegJointCount = 12;

    if (
      joint_names_.size() != kGo2LegJointCount ||
      motor_indices_.size() != kGo2LegJointCount ||
      signs_.size() != kGo2LegJointCount ||
      offsets_.size() != kGo2LegJointCount)
    {
      throw std::runtime_error(
        "joint_names, motor_indices, signs, and offsets must each contain "
        "exactly 12 entries.");
    }

    for (size_t i = 0; i < kGo2LegJointCount; ++i) {
      if (motor_indices_[i] < 0 || motor_indices_[i] >= 20) {
        throw std::runtime_error(
          "motor_indices[" + std::to_string(i) +
          "] must be in the LowState motor_state range [0, 19].");
      }

      if (joint_names_[i].empty()) {
        throw std::runtime_error(
          "joint_names[" + std::to_string(i) + "] must not be empty.");
      }

      if (std::abs(signs_[i]) < 1e-9) {
        throw std::runtime_error(
          "signs[" + std::to_string(i) + "] must not be zero.");
      }
    }
  }

  void on_lowstate(const unitree_go::msg::LowState::SharedPtr msg)
  {
    sensor_msgs::msg::JointState out;

    out.header.stamp = now();
    out.name = joint_names_;

    out.position.resize(joint_names_.size());
    out.velocity.resize(joint_names_.size());
    out.effort.resize(joint_names_.size());

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      const auto motor_index =
        static_cast<size_t>(motor_indices_[i]);

      const auto & motor = msg->motor_state[motor_index];

      out.position[i] = signs_[i] * motor.q + offsets_[i];
      out.velocity[i] = signs_[i] * motor.dq;
      out.effort[i] = signs_[i] * motor.tau_est;
    }

    joint_state_pub_->publish(out);
  }

  std::string lowstate_topic_;
  std::string joint_states_topic_;

  std::vector<std::string> joint_names_;
  std::vector<int64_t> motor_indices_;
  std::vector<double> signs_;
  std::vector<double> offsets_;

  rclcpp::Subscription<unitree_go::msg::LowState>::SharedPtr lowstate_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Go2JointStateBridge>());
  rclcpp::shutdown();
  return 0;
}