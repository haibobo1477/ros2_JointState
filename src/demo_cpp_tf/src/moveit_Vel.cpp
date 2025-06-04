#include <rclcpp/rclcpp.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/parameter_client.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <Eigen/Dense>



class JacobianSolver : public rclcpp::Node
{
public:
    JacobianSolver() : Node("jacobian_solver_client") 
    {    
        RCLCPP_INFO(this->get_logger(), "Start to compute jacobian matrix");
    }


    void wait_for_param_service(const std::shared_ptr<rclcpp::Node> & node)
    {
        auto param_client = std::make_shared<rclcpp::SyncParametersClient>(node, "/move_group");

        while (!param_client->wait_for_service(std::chrono::seconds(1))) 
            {
               RCLCPP_INFO(node->get_logger(), "Waiting for parameter service...");
            }

        auto desc = param_client->get_parameter<std::string>("robot_description");
        auto srdf_str = param_client->get_parameter<std::string>("robot_description_semantic");
        RCLCPP_INFO(node->get_logger(), "robot_description length = %ld", desc.size());
        RCLCPP_INFO(node->get_logger(), "robot_description_semantic length = %ld", srdf_str.size());


        this->declare_parameter("robot_description", desc);
        this->set_parameter(rclcpp::Parameter("robot_description", desc));
        this->declare_parameter("robot_description_semantic", srdf_str);
        this->set_parameter(rclcpp::Parameter("robot_description_semantic", srdf_str));


         // RobotModelLoader
        robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
        robot_model_ = robot_model_loader.getModel();

        if (!robot_model_) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load robot model.");
            return;
        }

       // JointModelGroup
        group_ = robot_model_->getJointModelGroup("interbotix_arm");

        if (!group_) 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to find joint group 'interbotix_arm'");
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Successfully loaded JointModelGroup: interbotix_arm");
        }

        // （可选）打印所有 group 名称，帮助你确认名字
        // auto group_names = robot_model_->getJointModelGroupNames();
        // RCLCPP_INFO(this->get_logger(), "Available JointModelGroups:");
        // for (const auto& name : group_names) 
        // {
        //     RCLCPP_INFO(this->get_logger(), " - %s", name.c_str());
        // }

        // LinkModel
        link_ = group_->getLinkModel("vx300s/ee_gripper_link");
        
        auto link_names = robot_model_->getLinkModelNames();
        for (const auto& name : link_names) 
        {
            RCLCPP_INFO(this->get_logger(), "Link: %s", name.c_str());
        }


        reference_point_position_ = Eigen::Vector3d(0.0, 0.0, 0.0);
        robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);  
        // 1. 创建 joint 值（根据你的 URDF 是几个关节）
        std::vector<double> joint_values = {0.0, 0.5, -0.3, 1.0, 0.2, 0.0};  // 后续需要将机械臂的状态实时传输过来，创建一个订阅者订阅话题

        // 2. 设置到 robot_state_
        robot_state_->setJointGroupPositions(group_, joint_values);   
        robot_state_->updateLinkTransforms();  // 确保链接坐标更新


        bool success = robot_state_->getJacobian(group_, link_, reference_point_position_, jacobian_);

        if (success) 
        {
            RCLCPP_INFO(this->get_logger(), "Jacobian computed:");
            std::cout << jacobian_ << std::endl;
        } 
        else 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to compute Jacobian!");
        }

       
    }



private:

   moveit::core::RobotModelPtr robot_model_;
   const moveit::core::JointModelGroup* group_;
   const moveit::core::LinkModel* link_;
   Eigen::Vector3d reference_point_position_;
   Eigen::MatrixXd jacobian_;
   moveit::core::RobotStatePtr robot_state_;
   
   rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
   
};



int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JacobianSolver>();
    node->wait_for_param_service(node);
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}












    // void setup()
    // {

    //     robot_model_loader::RobotModelLoader robot_model_loader(shared_from_this(), "robot_description");
    //     robot_model_ = robot_model_loader.getModel();
        
    //     if (!robot_model_) 
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "cannot upload robot_model!");
    //         return;
    //     }

    //     group = robot_model_->getJointModelGroup("interbotix_arm");

    //     if (!group) 
    //     {
    //         RCLCPP_ERROR(this->get_logger(), "cannot find planning group interbotix_arm");
    //     }

    // }