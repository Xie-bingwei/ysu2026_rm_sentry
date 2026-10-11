#ifndef MPC_CONTROLLER__MPC_SOLVER_HPP_
#define MPC_CONTROLLER__MPC_SOLVER_HPP_

#include <Eigen/Dense>
#include <osqp.h>
#include <vector>
#include <stdexcept>
#include "mpc_controller/mpc_model.hpp"

namespace mpc_controller
{
    struct MPC_Weights
    {   
        // 状态代价权重Q，Qf(终端代价), 控制代价权重R
        double q_pos;
        double q_vel;
        double q_terminal_pos;
        double q_terminal_vel;
        double r_acc;
    };
    
    class MPC_Solver
    {
        public:
        MPC_Solver(const MPC_Model &model, const MPC_Weights &w);
        ~MPC_Solver();

        // 禁用拷贝构造和拷贝赋值，防止不同对象指向同一块内存
        MPC_Solver(const MPC_Solver &) = delete;
        MPC_Solver & operator = (const MPC_Solver &) = delete;

        // 常量
        const Eigen::MatrixXd& H() {return H_;}             // Nm*Nm
        const Eigen::MatrixXd& Aineq() {return A_;}         // 4Nm*Nm(QP约束A向量)
        const Eigen::MatrixXd& Sv() {return Sv_;}           // Nm*Nn
        const Eigen::MatrixXd& Q_bar() {return Q_bar_;}     // Nn*Nn
        const Eigen::MatrixXd& R_bar() {return R_bar_;}     // Nm*Nm

        // 每拍更新
        // 计算f并写入osqp的q
        void updateCost(const Eigen::Vector4d &x0, const Eigen::VectorXd &x_ref);

        // 计算b，并写入osqp的上界u_
        void computeBounds(
        const Eigen::Vector4d &x0,
        const Eigen::Vector2d &u_max, const Eigen::Vector2d &u_min,
        const Eigen::Vector2d &v_max, const Eigen::Vector2d &v_min);

        const Eigen::VectorXd& f() {return f_;}

        // 求解
        bool solve();

        const Eigen::VectorXd& solution() const {return solution_;}            // 返回决策变量u
        Eigen::Vector2d firstControl() const {return solution_.head<2>();}     // 只返回第一个控制量

        // 过程诊断接口
        int lastIterations() const {return last_iter_;}       
        int lastStatus() const {return last_status_;}
        const std::vector<c_float>& bounds() const {return u_;}

        // 问题规模接口
        int runHorizon() {return N_;}
        int numVars() {return 2 * N_;}   // QP决策变量U维度, 控制输入为ux,uy, N个序列有2N个决策变量
        int numCons() {return 8 * N_;}   // QP约束向量b长度 -> 优化问题有8N个不等式约束

        private:
        const MPC_Model &model_;
        MPC_Weights w_;
        int N_;
        Eigen::MatrixXd Q_bar_, R_bar_, Sv_, H_, A_;
        Eigen::VectorXd f_;

        // OSQP
        void setupOSQP();
        OSQPWorkspace *work_ = nullptr;
        Eigen::VectorXd solution_;          // OSQP求解的是c_float*数组，需转化向量，使用solution_接受，对应预测控制量U(2*N_)
        int last_iter_ = 0;                 // OSQP迭代计数器

        // OSQP退出状态(OSQP_SOLVED-找到最优解，OSQP_MAX_ITER_REACHED-达到最大次数，OSQP_PRIMAL_INFEASIBLE-原问题不可行)
        int last_status_ = 0;                

        /* CSC数组 -- OSQP的稀疏矩阵形式
        x -- 每个非零元素的值(按列存储)，i -- 每个非零元素的行索引，p -- 每一列的起始位置,长度n+1
        q_ -> f_，l_ -> b下界，u_ -> b上界，P_ -> H矩阵，A_ -> Aineq_
        OSQP: min[0.5*(x^T)Px + (q^T)x]  s.t. l<=Ax<=u     当前建模: min[0.5*(U^T)HU + (f^T)U]  s.t. AU <= b 
        注: c_int类型为long long*/
        std::vector<c_float> P_x_, A_x_, q_, l_, u_;
        std::vector<c_int> P_i_, P_p_, A_i_, A_p_;
    };
}

#endif