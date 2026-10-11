#include "mpc_controller/mpc_controller.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"     
#include <algorithm>
#include <cmath>
#include <vector>

#include <tf2/exceptions.h>     // tf2::TransformException
#include <tf2/time.h>           // tf2::durationFromSec
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>   // tf2::doTransform(PoseStamped)

namespace mpc_controller
{
    void MPCController::configure(
            const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,      
            std::string name,
            std::shared_ptr<tf2_ros::Buffer> tf,
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) 
    {
        auto node = parent.lock();     // 弱指针转强指针
        node_ = parent;
        if(!node) {
            throw nav2_core::PlannerException("Unable to lock node");
        }
        tf_ = tf;
        costmap_ros_ = costmap_ros;
        plugin_name_ = name;
        logger_ = node -> get_logger();
        clock_ = node -> get_clock();

        // 声明horizon(预测步长N)和control_dt(Ts)
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".horizon", rclcpp::ParameterValue(20) 
        );
        nav2_util::declare_parameter_if_not_declared(
            node, plugin_name_ + ".control_dt", rclcpp::ParameterValue(0.05) // 20Hz
        );
        
        RCLCPP_INFO(logger_, "Configured MPCController plugin: %s", plugin_name_.c_str());

        // 使用lambda表达式简化后续读参数代码
        auto get_d = [&](const std::string &n, double def) {
            nav2_util::declare_parameter_if_not_declared(
                node, plugin_name_ + "." + n, rclcpp::ParameterValue(def));
            return node->get_parameter(plugin_name_ + "." + n).as_double();
        };
        auto get_i = [&](const std::string &n, int def) {
            nav2_util::declare_parameter_if_not_declared(
                node, plugin_name_ + "." + n, rclcpp::ParameterValue(def));
            return static_cast<int>(node->get_parameter(plugin_name_ + "." + n).as_int());
        };

        // 读MPC模型
        dt_ = get_d("control_dt", 0.05);    // 20Hz, dt = 1/controller_frequency 
        N_ = get_i("horizon", 20);          // 20步预测
        v_des_ = get_d("v_des", 1.0);       // 期望速度

        // 读MPC权重参数
        MPC_Weights weights;
        weights.q_pos = get_d("q_pos", 10.0);
        weights.q_vel = get_d("q_vel", 1.0);
        weights.q_terminal_pos = get_d("q_terminal_pos", 100.0);
        weights.q_terminal_vel = get_d("q_terminal_vel", 10.0);
        weights.r_acc = get_d("r_acc", 0.5);

        // 读速度，加速度约束
        const double a_lim = get_d("a_max", 3.0);
        const double v_lim = get_d("v_max", 2.0);
        a_max_ = Eigen::Vector2d(a_lim, a_lim);
        a_min_ = Eigen::Vector2d(-a_lim, -a_lim);
        v_max_ = Eigen::Vector2d(v_lim, v_lim);
        v_min_ = Eigen::Vector2d(-v_lim, -v_lim);

        mpc_model_  = std::make_unique<MPC_Model>(dt_, N_);
        mpc_solver_ = std::make_unique<MPC_Solver>(*mpc_model_, weights);

        RCLCPP_INFO(logger_, "MPC ready: N=%d dt=%.3f v_des=%.2f a_max=%.2f v_max=%.2f",
                    N_, dt_, v_des_, a_lim, v_lim);
    }

    void MPCController::cleanup() {RCLCPP_INFO(logger_, "Cleanup MPCController");}
    void MPCController::activate() {RCLCPP_INFO(logger_, "Activate MPCController");}
    void MPCController::deactivate() {RCLCPP_INFO(logger_, "Deactivate MPCController");}
    void MPCController::setPlan(const nav_msgs::msg::Path &path) {global_plan_ = path;}

    geometry_msgs::msg::TwistStamped MPCController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker */*goalchecker*/) 
    {   
        if (!mpc_solver_) {
            RCLCPP_WARN(logger_, "QP faild, status:%d  iter:%d  QP: n=%d m=%d",
                        mpc_solver_->lastStatus(), mpc_solver_->lastIterations(),
                        mpc_solver_->numVars(), mpc_solver_->numCons());
            throw nav2_core::PlannerException("MPC solver not initialized");
        }

        const double yaw = tf2::getYaw(pose.pose.orientation);

        /* MPC基于odom系，输出到机器人需要机体系 */
        // velocity是机体系，先旋到odom，基于yaw二维旋转矩阵
        Eigen::Vector4d x0;
        x0 << pose.pose.position.x,          // odom系下px
              pose.pose.position.y,          // odom py
              std::cos(yaw) * velocity.linear.x - std::sin(yaw) * velocity.linear.y,    // 机体系下的vx
              std::sin(yaw) * velocity.linear.x + std::cos(yaw) * velocity.linear.y;    // 机体 vy

        // 参考序列
        const Eigen::VectorXd x_ref = buildReference(pose);

        // 求解
        mpc_solver_->updateCost(x0, x_ref);
        mpc_solver_->computeBounds(x0, a_max_, a_min_, v_max_, v_min_);
        if (!mpc_solver_->solve()) {
            throw nav2_core::PlannerException("MPC QP failed");
        }

        // u0 -> 速度指令(odom系)
        const Eigen::Vector2d u0 = mpc_solver_->firstControl();
        const Eigen::Vector2d v_odom(x0[2] + dt_ * u0.x(),
                                     x0[3] + dt_ * u0.y());

        // odom -> 机体系，组装输出
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;        
        cmd_vel.twist.linear.x =  std::cos(yaw) * v_odom.x() + std::sin(yaw) * v_odom.y();
        cmd_vel.twist.linear.y = -std::sin(yaw) * v_odom.x() + std::cos(yaw) * v_odom.y();
        cmd_vel.twist.angular.z = 0.0;

        return cmd_vel;
    }

    void MPCController::setSpeedLimit(const double &, const bool &) {
        RCLCPP_INFO(logger_, "SpeedLimit waits for implementation!");
        return;
    }

    Eigen::VectorXd MPCController::buildReference(const geometry_msgs::msg::PoseStamped &pose)
    {
        Eigen::VectorXd x_ref(4 * N_);
        x_ref.setZero();
        
        // 路径转到odom系
        nav_msgs::msg::Path plan = transformPlanToOdom(pose);
        if (plan.poses.size() < 2) { 
            if (plan.poses.empty()) {     // 空路径，报错
                throw nav2_core::PlannerException("MPC: received empty plan, nothing to track");
            }

            if (plan.poses.size() == 1) {   // 单点路径，退化为目标点参考
                const auto & p = plan.poses.front().pose;
                for (int i = 0; i < N_; ++i) {
                    x_ref.segment<4>(4 * i) << p.position.x, p.position.y, 0.0, 0.0;
                }
                return x_ref;
            }
        }

        // 正常路径，时间参数化采样，s[k]->从路径起点到第k个点的弧长
        const std::vector<double> s = cumulativeDistances(plan);

        // 找到路径上离机器人最近的点使用s0接收对应弧长
        double s0 = 0.0, best = 1e18;
        for (size_t k = 0; k < plan.poses.size(); ++k) {
            const double dx = plan.poses[k].pose.position.x - pose.pose.position.x;
            const double dy = plan.poses[k].pose.position.y - pose.pose.position.y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best) { best = d2; s0 = s[k]; }
        }

        // si = s0 + i*T*v_des
        for (int i = 1; i <= N_; ++i) {
            geometry_msgs::msg::PoseStamped pi;
            if (!samplePathAt(plan, s, s0 + i * dt_ * v_des_, pi)) {
                pi = plan.poses.back();          // 超出末端 -> 用终点（末端速度参考自然为 0）
            }
            const double yaw_i = tf2::getYaw(pi.pose.orientation);
            x_ref.segment<4>(4 * (i - 1))
                << pi.pose.position.x, pi.pose.position.y,
                // 在路径点参考速度为当前切线方向，其夹角为yawi，则vx = v * cos(yawi), vy = v * sin(yawi)
                (i < N_ ? v_des_ : 0.0) * std::cos(yaw_i),
                (i < N_ ? v_des_ : 0.0) * std::sin(yaw_i);
        }
        return x_ref;
    }

    /*以下三个函数为AI从北极熊的转化方式中改过来的*/
    nav_msgs::msg::Path MPCController::transformPlanToOdom(const geometry_msgs::msg::PoseStamped &pose)
    {
        if (global_plan_.poses.empty()) {
            throw nav2_core::PlannerException("MPC: received plan with zero length");
        }

        const std::string odom_frame = costmap_ros_->getGlobalFrameID();   // "odom"

        // ★ 整条路径在同一个 frame、同一时刻，所以【只查一次】变换就够了
        geometry_msgs::msg::TransformStamped tf_msg;
        try {
            tf_msg = tf_->lookupTransform(
                odom_frame, global_plan_.header.frame_id,
                tf2::TimePointZero,                 // 取最新的变换，避免时间戳问题
                tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException & ex) {
            throw nav2_core::PlannerException(std::string("MPC: TF failed: ") + ex.what());
        }

        nav_msgs::msg::Path plan_odom;
        plan_odom.header.frame_id = odom_frame;
        plan_odom.header.stamp = pose.header.stamp;

        const double keep_dist = 3.0 * N_ * dt_ * v_des_;   // 只保留机器人附近的点

        for (const auto & p : global_plan_.poses) {
            geometry_msgs::msg::PoseStamped out;
            tf2::doTransform(p, out, tf_msg);       // ← 不会抛异常
            out.pose.position.z = 0.0;

            const double dx = out.pose.position.x - pose.pose.position.x;
            const double dy = out.pose.position.y - pose.pose.position.y;
            if (dx * dx + dy * dy > keep_dist * keep_dist) {
                continue;
            }
            plan_odom.poses.push_back(out);
        }

        if (plan_odom.poses.size() < 2) {
            throw nav2_core::PlannerException("MPC: plan has < 2 poses in odom frame");
        }
        return plan_odom;
    }

    std::vector<double> MPCController::cumulativeDistances(const nav_msgs::msg::Path &path) const
    {
        std::vector<double> cumulative_distances;
        cumulative_distances.push_back(0.0);

        for (size_t i = 1; i < path.poses.size(); ++i) {
            const auto & prev_pose = path.poses[i - 1].pose.position;
            const auto & curr_pose = path.poses[i].pose.position;
            double distance = hypot(curr_pose.x - prev_pose.x, curr_pose.y - prev_pose.y);
            cumulative_distances.push_back(cumulative_distances.back() + distance);
        }
        return cumulative_distances;
    }

    bool MPCController::samplePathAt(const nav_msgs::msg::Path &path, const std::vector<double> &s,
            double target_s, geometry_msgs::msg::PoseStamped &out) const
    {
        if (path.poses.empty() || s.empty()) {
            return false;
        }
        if (target_s <= 0.0) {
            out = path.poses.front();
            return true;
        }
        if (target_s >= s.back()) {
            return false;                 // ← 超出末端，让调用方用终点
        }

        auto it = std::lower_bound(s.begin(), s.end(), target_s);
        size_t index = std::distance(s.begin(), it);
        if (index == 0) {
            out = path.poses.front();
            return true;
        }

        const double ratio = (target_s - s[index - 1]) / (s[index] - s[index - 1]);
        const auto & p1 = path.poses[index - 1];
        const auto & p2 = path.poses[index];

        out.header = p2.header;
        out.pose.position.x = p1.pose.position.x + ratio * (p2.pose.position.x - p1.pose.position.x);
        out.pose.position.y = p1.pose.position.y + ratio * (p2.pose.position.y - p1.pose.position.y);
        out.pose.position.z = 0.0;
        out.pose.orientation = p2.pose.orientation;
        return true;
    }
}; 

// 告诉pluginlib系统，该类是一个插件，基类是nav2_core::Controller，可动态加载
PLUGINLIB_EXPORT_CLASS(mpc_controller::MPCController, nav2_core::Controller);
