#include "mpc_controller/mpc_solver.hpp"
#include <algorithm>
#include <cmath>

namespace mpc_controller
{
    MPC_Solver::MPC_Solver(const MPC_Model &model, const MPC_Weights &w)
     : model_(model), w_(w), N_(model.horizon())   
    {
        const int n = N_ * 2;    // 堆叠状态维度(只有位置)
        const int m = N_ * 2;    // 堆叠控制维度(速度)
        const auto &Gamma = model_.Gamma();

        // Q_bar = blkdiag[q_pos*I2, ..., q_terminal_pos*I2]，带 γ^i 衰减
        const double gam = std::clamp(w_.horizon_decay, 1e-3, 1.0);
        Q_bar_ = Eigen::MatrixXd::Zero(n, n);
        for(int i = 0; i < N_; ++i) {
            const double d = std::pow(gam, static_cast<double>(i));
            const double q = (i == N_ - 1) ? w_.q_terminal_pos : w_.q_pos;
            Q_bar_.block<2, 2>(2 * i, 2 * i) = (q * d) * Eigen::Matrix2d::Identity();
        }

        // R_bar = q_vel * I，跟踪参考速度
        R_bar_ = Eigen::MatrixXd::Zero(m, m);
        for(int i = 0; i < N_; ++i)
            R_bar_.block<2, 2>(2 * i, 2 * i) = w_.q_vel * Eigen::Matrix2d::Identity();

        // D: 块双对角，D(0,0)=I, D(k,k)=I, D(k,k-1)=-I
        D_ = Eigen::MatrixXd::Zero(m, m);
        for(int k = 0; k < N_; ++k) {
            D_.block<2, 2>(2 * k, 2 * k) = Eigen::Matrix2d::Identity();
            if(k > 0) D_.block<2, 2>(2 * k, 2 * (k - 1)) = -Eigen::Matrix2d::Identity();
        }

        // H = 2*(Gamma^T*Q_bar*Gamma + R_bar + r_rate*D^T*D)
        //      Gamma 每个非零块都是 dt*I2，所以 H 仍然对称正定，OSQP 只读上三角
        H_ = 2 * (Gamma.transpose() * Q_bar_ * Gamma + R_bar_
                  + w_.r_rate * D_.transpose() * D_);

        // A = [ I ; -I ; D ; -D ]   速度箱式 + 加速度(增量)约束
        A_ = Eigen::MatrixXd::Zero(8 * N_, m);
        A_.topRows(2 * N_)              =  Eigen::MatrixXd::Identity(m, m);
        A_.block(2 * N_, 0, 2 * N_, m)  = -Eigen::MatrixXd::Identity(m, m);
        A_.block(4 * N_, 0, 2 * N_, m)  =  D_;
        A_.block(6 * N_, 0, 2 * N_, m)  = -D_;

        setupOSQP();
    }
    
    MPC_Solver::~MPC_Solver()
    {
        if(work_) {
            osqp_cleanup(work_);
            work_ = nullptr;
        }
    }

    /*密集矩阵稀疏化*/
    // 只取上三角给H(即OSQP的P_)
    static void toUpperCSC(const Eigen::MatrixXd &M, 
                            std::vector<c_float> &x, std::vector<c_int> &i, std::vector<c_int> &p)
    {
        const int n = static_cast<int>(M.cols());
        x.clear(); i.clear(); p.assign(n + 1, 0);
        for(int j = 0; j < n; ++j) {            // 按列循环
            p[j] = static_cast<int>(i.size());
            for(int r = 0; r <= j; ++r) {      // 行<=列，只拉取上三角
                i.push_back(r);
                x.push_back(M(r, j));
            }
        }
        p[n] = static_cast<int>(i.size());
    }

    // 取全部元素给Aineq
    static void toFullCSC(const Eigen::MatrixXd &M,
                            std::vector<c_float> &x, std::vector<c_int> &i, std::vector<c_int> &p)
    {
        const int n = static_cast<int>(M.cols());
        const int m = static_cast<int>(M.rows());
        x.clear(); i.clear(); p.assign(n + 1, 0);
        for(int j = 0; j < n; ++j) {
            p[j] = static_cast<int>(i.size());
            for(int r = 0; r < m; ++r) {
                i.push_back(r);
                x.push_back(M(r, j));
            }
        }
        p[n] = static_cast<int>(i.size());
    }

    void MPC_Solver::setupOSQP()
    {
        const int n = numVars();    // 2N_   决策向量U*
        const int m = numCons();    // 8N_   约束向量b

        // 稠密转稀疏(CSC)
        toUpperCSC(H_, P_x_, P_i_, P_p_);
        toFullCSC(A_, A_x_, A_i_, A_p_);

        // 初始化q_(f), l_(因为只有上界，下界全inf), u_(b约束上界)
        q_.assign(n, 0.0);
        l_.assign(m, -OSQP_INFTY);
        u_.assign(m, 0.0);

        // 构造CSC结构体
        csc P_csc;
        P_csc.nzmax = static_cast<c_int>(P_x_.size());
        P_csc.n = n; P_csc.m = n;
        P_csc.p = P_p_.data(); P_csc.i = P_i_.data(); P_csc.x = P_x_.data();
        P_csc.nz = -1;

        csc A_csc;
        A_csc.nzmax = static_cast<c_int>(A_x_.size());
        A_csc.n = n; A_csc.m = m;
        A_csc.p = A_p_.data(); A_csc.i = A_i_.data(); A_csc.x = A_x_.data();
        A_csc.nz = -1;

        // 组装OSQPData
        OSQPData data;
        data.n = n; data.m = m;
        data.P = &P_csc; data.A = &A_csc;
        data.q = q_.data(); data.u = u_.data(); data.l = l_.data();

        // 设置求解器参数
        OSQPSettings st;
        osqp_set_default_settings(&st);
        st.verbose    = 0;       // 关闭求解器打印
        st.warm_start = 1;       // 开启暖启动
        st.polish     = 1;       // 求解后做抛光提高精度
        st.scaling    = 10;      // 迭代缩放次数
        st.eps_abs    = 1e-4;    // 绝对收敛容差
        st.eps_rel    = 1e-4;    // 相对收敛容差
        st.max_iter   = 2000;   // 迭代次数iter大约为250次

        if(osqp_setup(&work_, &data, &st) != 0) 
            throw std::runtime_error("MPC_Solver: osqp_setup failed");
    }
    
    void MPC_Solver::updateCost(const Eigen::Vector2d &x0, const Eigen::Vector2d &v_meas,
                                const Eigen::VectorXd &p_ref, const Eigen::VectorXd &v_ref)
    {
        // E = Phi*x0 - p_ref
        const Eigen::VectorXd E = model_.Phi() * x0 - p_ref;
        // u_{-1} = v_meas 是常数项，从 D*U 里挪到 f
        Eigen::VectorXd c = Eigen::VectorXd::Zero(numVars());
        c.head<2>() = v_meas;

        // f = 2*Gamma^T*Q_bar*E - 2*R_bar*v_ref - 2*r_rate*D^T*c
        f_ = 2.0 * model_.Gamma().transpose() * Q_bar_ * E
           - 2.0 * R_bar_ * v_ref
           - 2.0 * w_.r_rate * D_.transpose() * c;

        for(int i = 0; i < numVars(); ++i)
            q_[i] = f_[i];
        osqp_update_lin_cost(work_, q_.data());
    }
    
    void MPC_Solver::computeBounds(
        const Eigen::Vector2d &v_meas,
        const Eigen::Vector2d &u_max, const Eigen::Vector2d &u_min,
        const Eigen::Vector2d &a_max)
    {
        const double dt = model_.dt();
        const Eigen::Vector2d du = dt * a_max;      // 每拍允许的速度变化量

        Eigen::Map<Eigen::VectorXd> b(u_.data(), numCons());

        for(int i = 0; i < N_; ++i) {
            // (1)(2) 速度箱式约束  v_min <= u_i <= v_max
            b.segment<2>(2 * i)           = u_max;
            b.segment<2>(2 * N_ + 2 * i)  = -u_min;

            // (3)(4) 加速度约束 |u_i - u_{i-1}| <= dt*a_max,  u_{-1} = v_meas
            if(i == 0) {
                b.segment<2>(4 * N_ + 2 * i) =  v_meas + du;   //  u_0 <= v_meas + dt*a_max
                b.segment<2>(6 * N_ + 2 * i) = -v_meas + du;   // -u_0 <= -v_meas + dt*a_max
            } else {
                b.segment<2>(4 * N_ + 2 * i) = du;
                b.segment<2>(6 * N_ + 2 * i) = du;
            }
        }
        osqp_update_bounds(work_, l_.data(), u_.data());
    }

    bool MPC_Solver::solve()
    {
        // 暖启动在计算之前，solution->x上次原始解(U*,长度2N_)，solution->y上次对偶解(拉格朗日乘子,长度8N_)
        osqp_warm_start(work_, work_->solution->x, work_->solution->y);
        
        if(osqp_solve(work_) != 0) 
            return false;
        
        // 成功则提取状态和迭代次数
        last_status_ = static_cast<int>(work_->info->status_val);
        last_iter_ = static_cast<int>(work_->info->iter);

        // 1 = OSQP_SOLVED(找到高精度最优解), 2 = OSQP_SOLVED_INACCURATE(找到低精度可用解)
        if(last_status_!= OSQP_SOLVED && last_status_ != OSQP_SOLVED_INACCURATE) 
            return false;

        solution_ = Eigen::Map<Eigen::VectorXd>(work_->solution->x, numVars());
        return true;
    }
}
