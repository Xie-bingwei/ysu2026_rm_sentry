#include "mpc_controller/mpc_solver.hpp"

namespace mpc_controller
{
    MPC_Solver::MPC_Solver(const MPC_Model &model, const MPC_Weights &w)
     : model_(model), w_(w), N_(model.horizon())   
    {
        const int n = N_ * 4;    // 堆叠状态维度
        const int m = N_ * 2;    // 堆叠控制维度
        const auto &Gamma = model.Gamma();

        // Q_bar -> blkdiag[Q,Q,Q......,QN]
        Eigen::Matrix4d Q = Eigen::Vector4d(w_.q_pos, w_.q_pos, w_.q_vel, w_.q_vel).asDiagonal();  // asDiagonal()将向量作为对角阵使用
        Eigen::Matrix4d QN = Eigen::Vector4d(w_.q_terminal_pos, w_.q_terminal_pos, w_.q_terminal_vel, w_.q_terminal_vel).asDiagonal();
        Q_bar_ = Eigen::MatrixXd::Zero(n, n);
        for(int i = 0; i < N_; ++i) 
            Q_bar_.block<4, 4>(4 * i, 4 * i) = (i == N_ - 1) ? QN : Q;

        // R_bar -> blkdiag[R,R,R......,R]Q_bar_
        Eigen::Matrix2d R = Eigen::Vector2d(w_.r_acc, w_.r_acc).asDiagonal();
        R_bar_ = Eigen::MatrixXd::Zero(m , m);
        for(int i = 0; i < N_; ++i)
            R_bar_.block<2, 2>(2 * i, 2 * i) = R;

        //  Sv = [[0,0,1,0],    
        //        [0,0,0,1]] ⊗ I_N, 从堆叠态抽出vx,xy
        Sv_ = Eigen::MatrixXd::Zero(m, n);
        for(int i = 0; i < N_; ++i) {
            Sv_(2 * i, 4 * i + 2) = 1.0;
            Sv_(2 * i + 1, 4 * i + 3) = 1.0;
        }

        // H = 2(Gamma^T*Q_bar_*Gamma + R_bar_)
        H_ = 2 * (Gamma.transpose() * Q_bar_ * Gamma + R_bar_);

        // A = [I,-I,Sv_*Gamma, -Sv_*Gamma]
        A_ = Eigen::MatrixXd::Zero(8 * N_, m);
        A_.topRows(2 * N_) = Eigen::MatrixXd::Identity(m , m);
        A_.block(2 * N_, 0, 2 * N_, m) = -Eigen::MatrixXd::Identity(m, m);
        A_.block(4 * N_, 0, 2 * N_, m) = Sv_ * Gamma;
        A_.block(6 * N_, 0, 2 * N_, m) = -Sv_ * Gamma;

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
    
    void MPC_Solver::updateCost(const Eigen::Vector4d &x0, const Eigen::VectorXd &x_ref)
    {
        // E = Phi * x0 - X_ref,     f = 2 * Gamma^T * Q_bar * E
        const Eigen::VectorXd E = model_.Phi() * x0 - x_ref;
        f_ = 2.0 * model_.Gamma().transpose() * Q_bar_ * E;

        // 写入osqp
        for(int i = 0; i < numVars(); ++i)
            q_[i] = f_[i];
        osqp_update_lin_cost(work_, q_.data());
    }
    
    void MPC_Solver::computeBounds(
        const Eigen::Vector4d &x0,
        const Eigen::Vector2d &u_max, const Eigen::Vector2d &u_min,
        const Eigen::Vector2d &v_max, const Eigen::Vector2d &v_min)
    {
        Eigen::Map<Eigen::VectorXd> b(u_.data(), numCons());
        
        // 加速度(输入)约束，segment<2>(i)，从i位置连续取2个向量块
        for(int i = 0; i < N_; ++i) {
            b.segment<2>(2 * i) = u_max;               // 令前2N_个元素为u_max
            b.segment<2>(2 * N_ + 2 * i) = -u_min;     // 令2N_ - 4N_个元素为u_min
        }

        // 速度约束
        Eigen::VectorXd SvPhix0 = Sv_ * (model_.Phi() * x0);
        for(int i = 0; i < N_; ++i) {
            // 左边是2*1向量，右边也需注意使用2*1(SvPhix0是2N*1)
            b.segment<2>(4 * N_ + 2 * i) = v_max - SvPhix0.segment<2>(2 * i);
            b.segment<2>(6 * N_ + 2 * i) = -v_min + SvPhix0.segment<2>(2 * i);
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
