#include <array>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>
#include <sstream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

class LegOdometryNode : public rclcpp::Node
{
public:
    LegOdometryNode() : Node("go2_leg_odometry")
    {
        declare_parameter<std::string>("joint_states_topic", "/joint_states");
        declare_parameter<std::string>("foot_contacts_topic", "/foot_contacts");
        declare_parameter<std::string>("imu_topic", "/imu/data");
        declare_parameter<std::string>("odom_topic", "/odometry/leg");
        declare_parameter<std::string>("base_link", "base_link");
        declare_parameter<std::string>("odom_frame", "odom");
        declare_parameter<std::vector<std::string>>(
            "foot_links",
            std::vector<std::string>{"FR_foot", "FL_foot", "RR_foot", "RL_foot"});
        declare_parameter<double>("contact_min_count", 1.0);

        joint_states_topic_ = get_parameter("joint_states_topic").as_string();
        foot_contacts_topic_ = get_parameter("foot_contacts_topic").as_string();
        imu_topic_ = get_parameter("imu_topic").as_string();
        odom_topic_ = get_parameter("odom_topic").as_string();
        base_link_ = get_parameter("base_link").as_string();
        odom_frame_ = get_parameter("odom_frame").as_string();
        foot_links_ = get_parameter("foot_links").as_string_array();

        std::string robot_description;
        if (!this->get_parameter_or("robot_description", robot_description, std::string("")))
        {
            robot_description = "";
        }
        if (robot_description.empty())
        {
            if (!this->has_parameter("robot_description"))
            {
                this->declare_parameter<std::string>("robot_description", "");
            }
            robot_description = this->get_parameter("robot_description").as_string();
        }

        if (robot_description.empty())
        {
            throw std::runtime_error("robot_description parameter is empty");
        }

        try
        {
            pinocchio::urdf::buildModelFromXML(robot_description, model_);
            data_ = std::make_unique<pinocchio::Data>(model_);
        }
        catch (const std::exception &e)
        {
            throw std::runtime_error(std::string("Failed to build Pinocchio model: ") + e.what());
        }

        q_ = Eigen::VectorXd::Zero(model_.nq);

        base_frame_id_ = model_.getFrameId(base_link_);
        if (base_frame_id_ == static_cast<pinocchio::FrameIndex>(-1))
        {
            throw std::runtime_error("Base frame not found in Pinocchio model: " + base_link_);
        }

        for (size_t i = 0; i < foot_links_.size(); ++i)
        {
            auto frame_id = model_.getFrameId(foot_links_[i]);
            if (frame_id == static_cast<pinocchio::FrameIndex>(-1))
            {
                throw std::runtime_error("Foot frame not found in Pinocchio model: " + foot_links_[i]);
            }
            foot_frame_ids_.push_back(frame_id);
            // RCLCPP_INFO(get_logger(), "Foot frame %zu: %s -> id=%u",
            //             i, foot_links_[i].c_str(), static_cast<unsigned int>(frame_id));
        }

        for (pinocchio::JointIndex jid = 1; jid < model_.njoints; ++jid)
        {
            const auto &jmodel = model_.joints[jid];
            const std::string &jname = model_.names[jid];
            if (jmodel.nq() == 1)
            {
                joint_name_to_q_index_[jname] = jmodel.idx_q();
                // RCLCPP_INFO(get_logger(), "Pinocchio joint: %s idx_q=%d",
                //             jname.c_str(), static_cast<int>(jmodel.idx_q()));
            }
        }

        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic_, 50);

        joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            joint_states_topic_, 50,
            std::bind(&LegOdometryNode::jointCallback, this, std::placeholders::_1));

        contact_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
            foot_contacts_topic_, 50,
            std::bind(&LegOdometryNode::contactCallback, this, std::placeholders::_1));

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS(),
            std::bind(&LegOdometryNode::imuCallback, this, std::placeholders::_1));

        last_pos_.fill({0.0, 0.0, 0.0});
        have_last_pos_.fill(false);
        contacts_.fill(0);

        RCLCPP_INFO(get_logger(), "go2_leg_odometry started with Pinocchio");
    }

private:
    struct Vec3
    {
        double x, y, z;
    };

    void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // for (size_t i = 0; i < std::min<size_t>(msg->name.size(), 12); ++i)
        // {
        //     if (i < msg->position.size())
        //     {
        //         RCLCPP_INFO(get_logger(), "joint_states[%zu]: %s = %.4f",
        //                     i, msg->name[i].c_str(), msg->position[i]);
        //     }
        // }
        latest_joint_state_ = msg;
        tryPublish();
    }

    void contactCallback(const std_msgs::msg::Int32MultiArray::SharedPtr msg)
    {
        // std::ostringstream oss;
        // oss << "foot_contacts: [";
        // for (size_t i = 0; i < msg->data.size(); ++i)
        // {
        //     if (i) oss << ", ";
        //     oss << msg->data[i];
        // }
        // oss << "]";
        // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

        for (size_t i = 0; i < 4 && i < msg->data.size(); ++i)
        {
            contacts_[i] = msg->data[i];
        }
        tryPublish();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        latest_imu_ = msg;
        tryPublish();
    }

    bool fillConfigurationFromJointState()
    {
        if (!latest_joint_state_)
        {
            RCLCPP_INFO(get_logger(), "No latest_joint_state_");
            return false;
        }

        q_.setZero();

        const size_t n = std::min(latest_joint_state_->name.size(), latest_joint_state_->position.size());
        for (size_t i = 0; i < n; ++i)
        {
            auto it = joint_name_to_q_index_.find(latest_joint_state_->name[i]);
            if (it == joint_name_to_q_index_.end())
            {
                RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                                     "JointState joint not found in Pinocchio model: %s",
                                     latest_joint_state_->name[i].c_str());
                continue;
            }
            q_[it->second] = latest_joint_state_->position[i];
        }

        return true;
    }

    bool computeFootPosition(size_t leg_idx, Vec3 &out)
    {
        if (!fillConfigurationFromJointState())
            return false;

        try
        {
            pinocchio::forwardKinematics(model_, *data_, q_);
            pinocchio::updateFramePlacements(model_, *data_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(get_logger(), "Pinocchio FK failed: %s", e.what());
            return false;
        }

        const auto foot_frame_id = foot_frame_ids_[leg_idx];
        const auto &oMf_foot = data_->oMf[foot_frame_id];
        const auto &oMf_base = data_->oMf[base_frame_id_];

        Eigen::Vector3d foot_in_base = oMf_base.actInv(oMf_foot).translation();

        // RCLCPP_INFO(get_logger(), "Foot %zu pos in %s: %.4f %.4f %.4f",
        //             leg_idx, base_link_.c_str(),
        //             foot_in_base.x(), foot_in_base.y(), foot_in_base.z());

        out.x = foot_in_base.x();
        out.y = foot_in_base.y();
        out.z = foot_in_base.z();
        return true;
    }

    Vec3 rotateByQuaternion(const geometry_msgs::msg::Quaternion &q, const Vec3 &v)
    {
        const double x = q.x, y = q.y, z = q.z, w = q.w;

        const double xx = x * x, yy = y * y, zz = z * z;
        const double xy = x * y, xz = x * z, yz = y * z;
        const double wx = w * x, wy = w * y, wz = w * z;

        Vec3 out;
        out.x = (1 - 2 * (yy + zz)) * v.x + 2 * (xy - wz) * v.y + 2 * (xz + wy) * v.z;
        out.y = 2 * (xy + wz) * v.x + (1 - 2 * (xx + zz)) * v.y + 2 * (yz - wx) * v.z;
        out.z = 2 * (xz - wy) * v.x + 2 * (yz + wx) * v.y + (1 - 2 * (xx + yy)) * v.z;
        return out;
    }

    void tryPublish()
    {
        if (!latest_joint_state_ || !latest_imu_)
        {
            RCLCPP_INFO(get_logger(), "failed to get !latest_joint_state_ || !latest_imu_");
            return;
        }

        const rclcpp::Time stamp =
            (latest_joint_state_->header.stamp.nanosec == 0 && latest_joint_state_->header.stamp.sec == 0)
                ? now()
                : rclcpp::Time(latest_joint_state_->header.stamp);

        if (!have_time_)
        {
            RCLCPP_INFO(get_logger(), "No have_time_");
            last_stamp_ = stamp;
            have_time_ = true;
            for (size_t i = 0; i < 4; ++i)
            {
                Vec3 p;
                if (computeFootPosition(i, p))
                {
                    last_pos_[i] = p;
                    have_last_pos_[i] = true;
                }
            }
            return;
        }

        const double dt = (stamp - last_stamp_).seconds();
        if (dt <= 1e-4)
            return;

        Vec3 vel_body{0.0, 0.0, 0.0};
        int used_contacts = 0;

        for (size_t i = 0; i < 4; ++i)
        {
            Vec3 p;
            if (!computeFootPosition(i, p))
            {
                RCLCPP_INFO(get_logger(), "computeFootPosition returned false");
                continue;
            }

            // std::ostringstream oss;
            // oss << "contacts_[" << i << "]: " << contacts_[i]
            //     << " have_last_pos[" << i << "]: " << have_last_pos_[i];
            // RCLCPP_INFO(get_logger(), "%s", oss.str().c_str());

            if (contacts_[i] > 0 && have_last_pos_[i])
            {
                Vec3 dp{
                    (p.x - last_pos_[i].x) / dt,
                    (p.y - last_pos_[i].y) / dt,
                    (p.z - last_pos_[i].z) / dt};

                vel_body.x += -dp.x;
                vel_body.y += -dp.y;
                vel_body.z += -dp.z;
                used_contacts++;
            }

            last_pos_[i] = p;
            have_last_pos_[i] = true;
        }

        if (used_contacts == 0)
        {
            RCLCPP_INFO(get_logger(), "no contact has been made");
            last_stamp_ = stamp;
            return;
        }

        vel_body.x /= used_contacts;
        vel_body.y /= used_contacts;
        vel_body.z /= used_contacts;

        // Vec3 vel_world = rotateByQuaternion(latest_imu_->orientation, vel_body);

        x_ += vel_body.x * dt;
        y_ += vel_body.y * dt;
        z_ += vel_body.z * dt;

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = stamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_link_;

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = z_;
        odom.pose.pose.orientation = latest_imu_->orientation;

        odom.twist.twist.linear.x = vel_body.x;
        odom.twist.twist.linear.y = vel_body.y;
        odom.twist.twist.linear.z = vel_body.z;
        odom.twist.twist.angular = latest_imu_->angular_velocity;

        odom.pose.covariance[0] = 0.05;
        odom.pose.covariance[7] = 0.05;
        odom.pose.covariance[14] = 0.10;
        odom.pose.covariance[21] = 0.20;
        odom.pose.covariance[28] = 0.20;
        odom.pose.covariance[35] = 0.20;

        odom.twist.covariance[0] = 0.05;
        odom.twist.covariance[7] = 0.05;
        odom.twist.covariance[14] = 0.10;
        odom.twist.covariance[21] = 0.10;
        odom.twist.covariance[28] = 0.10;
        odom.twist.covariance[35] = 0.10;

        odom_pub_->publish(odom);
        last_stamp_ = stamp;
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr contact_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
    sensor_msgs::msg::Imu::SharedPtr latest_imu_;

    std::string joint_states_topic_;
    std::string foot_contacts_topic_;
    std::string imu_topic_;
    std::string odom_topic_;
    std::string base_link_;
    std::string odom_frame_;

    std::vector<std::string> foot_links_;

    pinocchio::Model model_;
    std::unique_ptr<pinocchio::Data> data_;
    Eigen::VectorXd q_;
    pinocchio::FrameIndex base_frame_id_;
    std::vector<pinocchio::FrameIndex> foot_frame_ids_;
    std::unordered_map<std::string, int> joint_name_to_q_index_;

    std::array<int, 4> contacts_;
    std::array<Vec3, 4> last_pos_;
    std::array<bool, 4> have_last_pos_;

    bool have_time_{false};
    rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};

    double x_{0.0};
    double y_{0.0};
    double z_{0.0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegOdometryNode>());
    rclcpp::shutdown();
    return 0;
}