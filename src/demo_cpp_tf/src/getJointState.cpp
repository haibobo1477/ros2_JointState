#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <map>
#include <memory>
#include <cmath>  // for std::isnan

class JointStateMonitor : public rclcpp::Node
{
public:
    JointStateMonitor() : Node("joint_state_monitor")
    {
        RCLCPP_INFO(this->get_logger(), "Starting Joint State Monitor Node...");

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/vx300s/joint_states", 10,
            std::bind(&JointStateMonitor::jointStateCallback, this, std::placeholders::_1));

        // 创建定时器，每0.2秒执行一次打印任务
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&JointStateMonitor::timerCallback, this));
    }

private:
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        latest_msg_ = msg;
    }

    void timerCallback()
    {
        if (!latest_msg_)
            return;

        RCLCPP_INFO(this->get_logger(), "Latest JointState message with %lu joints", latest_msg_->name.size());

        for (size_t i = 0; i < latest_msg_->name.size(); ++i)
        {
            std::string joint_name = latest_msg_->name[i];
            double position = (i < latest_msg_->position.size()) ? latest_msg_->position[i] : 0.0;
            double velocity = (i < latest_msg_->velocity.size() && !std::isnan(latest_msg_->velocity[i])) ? latest_msg_->velocity[i] : 0.0;
            double effort   = (i < latest_msg_->effort.size()   && !std::isnan(latest_msg_->effort[i]))   ? latest_msg_->effort[i]   : 0.0;

            RCLCPP_INFO(this->get_logger(),
                        "Joint: %-20s | Pos: %8.4f | Vel: %8.4f | Eff: %8.4f",
                        joint_name.c_str(), position, velocity, effort);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::JointState::SharedPtr latest_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateMonitor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}







// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/joint_state.hpp>
// #include <string>
// #include <map>

// class JointStateMonitor : public rclcpp::Node
// {
// public:
//     JointStateMonitor() : Node("joint_state_monitor")
//     {
//         RCLCPP_INFO(this->get_logger(), "Starting Joint State Monitor Node...");

//         joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
//             "/vx300s/joint_states", 10,
//             std::bind(&JointStateMonitor::jointStateCallback, this, std::placeholders::_1));
//     }

// private:
//     void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
//     {
//         RCLCPP_INFO(this->get_logger(), "Received JointState message with %lu joints", msg->name.size());

//         for (size_t i = 0; i < msg->name.size(); ++i)
//         {
//             std::string joint_name = msg->name[i];
//             double position = (i < msg->position.size()) ? msg->position[i] : 0.0;
//             double velocity = (i < msg->velocity.size()) ? msg->velocity[i] : 0.0;
//             double effort   = (i < msg->effort.size())   ? msg->effort[i]   : 0.0;

//             RCLCPP_INFO(this->get_logger(),
//                         "Joint: %-20s | Pos: %8.4f | Vel: %8.4f | Eff: %8.4f",
//                         joint_name.c_str(), position, velocity, effort);
//         }
//     }

//     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<JointStateMonitor>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
