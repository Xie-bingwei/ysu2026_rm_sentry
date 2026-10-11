#ifndef MPC_CONTROLLER__MPC_TROLLER_HPP_
#define MPC_CONTROLLER__MPC_TROLLER_HPP_

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <vector>
#include "tf2/utils.h"

#include "mpc_controller/mpc_model.hpp"
#include "mpc_controller/mpc_solver.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace mpc_controller 
{
    class MPCController : public nav2_core :: Controller
    {
        public:
        MPCController() = default;
        ~MPCController() override = default;

        // 插件加载时调用，用于初始化参数
        void configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,      
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override;

        void cleanup() override;        // 清理资源，关闭OSQP求解器
        void activate() override;       // 激活控制器，开始接收路径
        void deactivate() override;     // 停用控制器，停止输出速度

        // 接收全局路径
        void setPlan(const nav_msgs::msg::Path &path) override;

        // 动态限速，percentage=True表示speed_limit是百分比，为False表示speed_limit是绝对值
        void setSpeedLimit(const double &speed_limit, const bool &percentage) override;
        
        // 核心控制循环，完成MPC求解返回速度指令
        geometry_msgs::msg::TwistStamped computeVelocityCommands(
            const geometry_msgs::msg::PoseStamped &pose,
            const geometry_msgs::msg::Twist &velocity,
            nav2_core::GoalChecker *goalchecker) override; 

        private:
        rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
        std::string plugin_name_;
        rclcpp::Logger logger_{rclcpp::get_logger("MPCController")};
        rclcpp::Clock::SharedPtr clock_;
        nav_msgs::msg::Path global_plan_;

        std::unique_ptr<MPC_Model> mpc_model_;
        std::unique_ptr<MPC_Solver> mpc_solver_;
        int N_ = 20;                       // 预测步长
        double dt_ = 0.05;                 // 采样时间
        double v_des_ = 1.0;               // 参考速度
        Eigen::Vector2d a_min_, a_max_;    // 加速度限幅值
        Eigen::Vector2d v_min_, v_max_;    // 速度限幅值

        // 参考序列x_ref生成
        Eigen::VectorXd buildReference(const geometry_msgs::msg::PoseStamped &pose);

        // 把全局路径从map转到odom(使用pb2025现有的)
        nav_msgs::msg::Path transformPlanToOdom(const geometry_msgs::msg::PoseStamped &pose);

        // 计算每个点的累积弧长
        std::vector<double> cumulativeDistances(const nav_msgs::msg::Path &path) const;

        // 按弧长取点
        bool samplePathAt(const nav_msgs::msg::Path &path, const std::vector<double> &s,
            double target_s, geometry_msgs::msg::PoseStamped &out) const;
    };
}  

#endif