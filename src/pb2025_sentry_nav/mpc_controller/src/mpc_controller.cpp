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
        dt_ = get_d("control_dt", 0.05);        // 20Hz, dt = 1/controller_frequency 
        N_ = get_i("horizon", 20);              // 20步预测
        v_des_ = get_d("v_des", 1.0);           // 期望速度
        a_lat_max_ = get_d("a_lat_max", 25.0);  // 2.0横向加速度限制来曲率限速

        // 读MPC权重参数
        MPC_Weights weights;
        weights.q_pos = get_d("q_pos", 100.0);
        weights.q_terminal_pos = get_d("q_terminal_pos", 200.0);
        weights.q_vel = get_d("q_vel", 5.0);            // 原 q_vel，现在作用在 u 上
        weights.r_rate = get_d("r_rate", 5.0);          // 原 r_acc
        weights.horizon_decay = get_d("horizon_decay", 1.0);

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
    void MPCController::setPlan(const nav_msgs::msg::Path &path) {global_plan_ = path; s_prev_ = 0.0;}

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

        v_ref_ = Eigen::VectorXd::Zero(2 * N_);
        const double yaw = tf2::getYaw(pose.pose.orientation);

        // 状态只剩位置(odom)
        const Eigen::Vector2d x0(pose.pose.position.x, pose.pose.position.y);

        // 实测速度转 odom，只喂给加速度约束的起点 u_{-1}
        // ★ 必须夹到 v_max 内：否则 u_0 >= v_meas - dt*a_max 和 u_0 <= v_max 冲突 -> PRIMAL_INFEASIBLE
        Eigen::Vector2d v_meas(std::cos(yaw) * velocity.linear.x - std::sin(yaw) * velocity.linear.y,
                               std::sin(yaw) * velocity.linear.x + std::cos(yaw) * velocity.linear.y);
        v_meas = v_meas.cwiseMax(-v_max_).cwiseMin(v_max_);

        const Eigen::VectorXd p_ref = buildReference(pose);   // 位置参考(2N)，速度参考填进 v_ref_

        mpc_solver_->updateCost(x0, v_meas, p_ref, v_ref_);
        mpc_solver_->computeBounds(v_meas, v_max_, v_min_, a_max_);
        if (!mpc_solver_->solve()) {
            throw nav2_core::PlannerException("MPC QP failed");
        }

        // ★ u0 就是 odom 系的期望速度，不再有 x0[2]+dt*u0 那个结构
        const Eigen::Vector2d u0 = mpc_solver_->firstControl();

        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header = pose.header;
        cmd_vel.twist.linear.x =  std::cos(yaw) * u0.x() + std::sin(yaw) * u0.y();
        cmd_vel.twist.linear.y = -std::sin(yaw) * u0.x() + std::cos(yaw) * u0.y();
        cmd_vel.twist.angular.z = 0.0;

        return cmd_vel;
    }

    void MPCController::setSpeedLimit(const double &, const bool &) {
        RCLCPP_INFO(logger_, "SpeedLimit waits for implementation!");
        return;
    }

    Eigen::VectorXd MPCController::buildReference(const geometry_msgs::msg::PoseStamped &pose)
    {
        Eigen::VectorXd x_ref(2 * N_);
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
                    x_ref.segment<2>(2 * i) << p.position.x, p.position.y;
                    v_ref_.segment<2>(2 * i) << 0.0, 0.0;
                }
                return x_ref;
            }
        }

        // 正常路径，时间参数化采样，s[k]->从路径起点到第k个点的弧长
        const std::vector<double> s = cumulativeDistances(plan);

        // 找到路径上离机器人最近的点使用s0接收对应弧长
        const double back_margin = 0.5;                 // 允许 0.5 m 回退
        size_t k_begin = 0;
        while (k_begin + 1 < s.size() && s[k_begin] < s_prev_ - back_margin) ++k_begin;

        double s0 = s_prev_, best = 1e18;
        for (size_t k = k_begin; k < plan.poses.size(); ++k) {
            const double dx = plan.poses[k].pose.position.x - pose.pose.position.x;
            const double dy = plan.poses[k].pose.position.y - pose.pose.position.y;
            const double d2 = dx * dx + dy * dy;
            if (d2 < best) { best = d2; s0 = s[k]; }
        }
        s_prev_ = s0;

        // 用相邻采样点差分求切向，不再依赖path里恒为 identity的orientation
        const double ds = 0.05;                    // 差分步长 [m]
        const double s_total = s.back();

        // 诊断累积量（循环前声明）
        double v_ref_min   = v_des_;
        double kappa_max   = 0.0;
        int    pinned_cnt  = 0;

        for (int i = 1; i <= N_; ++i) {
            const double si = s0 + i * dt_ * v_des_;

            geometry_msgs::msg::PoseStamped pi;
            bool at_end = false;
            if (!samplePathAt(plan, s, si, pi)) {
                pi = plan.poses.back();
                at_end = true;                     // 标记 已到路径末端
                ++pinned_cnt;                      // 统计被钉在终点的参考点数
            }

            geometry_msgs::msg::PoseStamped pa, pb;
            if (!samplePathAt(plan, s, std::max(0.0, si - ds), pa)) {pa = plan.poses.front();}
            if (!samplePathAt(plan, s, si + ds, pb)) {pb = plan.poses.back();}

            double tx = pb.pose.position.x - pa.pose.position.x;
            double ty = pb.pose.position.y - pa.pose.position.y;
            const double tn = std::hypot(tx, ty);
            if (tn > 1e-6) { tx /= tn; ty /= tn; } else { tx = 1.0; ty = 0.0; }

            // 终点刹车曲线
            const double s_remain = std::max(0.0, s_total - si);
            double v_ref = at_end ? 0.0 : v_des_;
            v_ref = std::min(v_ref, std::sqrt(2.0 * a_max_.x() * s_remain));

            // ★ 曲率限速  v <= sqrt(a_lat_max / kappa)
            {
                const double ds_c = 0.3;   // [m] 比切向步长大，抗离散化噪声
                geometry_msgs::msg::PoseStamped pc1, pc2, pc3;
                if (!samplePathAt(plan, s, std::max(0.0, si - ds_c), pc1)) pc1 = plan.poses.front();
                if (!samplePathAt(plan, s, si,                     pc2)) pc2 = plan.poses.back();
                if (!samplePathAt(plan, s, si + ds_c,              pc3)) pc3 = plan.poses.back();

                const double ax = pc2.pose.position.x - pc1.pose.position.x;
                const double ay = pc2.pose.position.y - pc1.pose.position.y;
                const double bx = pc3.pose.position.x - pc2.pose.position.x;
                const double by = pc3.pose.position.y - pc2.pose.position.y;

                const double la = std::hypot(ax, ay);
                const double lb = std::hypot(bx, by);
                const double lc = std::hypot(ax + bx, ay + by);
                const double cr = ax * by - ay * bx;

                const double kappa = (la * lb * lc > 1e-9)
                                   ? 2.0 * std::abs(cr) / (la * lb * lc) : 0.0;

                kappa_max = std::max(kappa_max, kappa);        // ★② 累积最大曲率

                if (kappa > 1e-6) {
                    v_ref = std::min(v_ref, std::sqrt(a_lat_max_ / kappa));
                }
            }

            v_ref_min = std::min(v_ref_min, v_ref);            // 累积最小参考速度

            x_ref.segment<2>(2 * (i - 1)) << pi.pose.position.x, pi.pose.position.y;
            v_ref_.segment<2>(2 * (i - 1)) << v_ref * tx, v_ref * ty;
        }

        // 诊断日志（循环之后、return 之前）
        RCLCPP_INFO_THROTTLE(logger_, *clock_, 500,
            "s0=%.2f s_tot=%.2f cover=%.2f | v_ref_min=%.2f v_des=%.2f | "
            "kappa_max=%.3f (r_min=%.2f m) | pinned=%d/%d",
            s0, s_total, N_ * dt_ * v_des_,
            v_ref_min, v_des_,
            kappa_max, (kappa_max > 1e-6 ? 1.0 / kappa_max : 999.0),
            pinned_cnt, N_);

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
