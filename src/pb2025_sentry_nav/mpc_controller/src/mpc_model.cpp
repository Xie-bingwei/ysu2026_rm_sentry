#include "mpc_controller/mpc_model.hpp"
/* 使用ZOH离散化系统模型  
 *  Ad = [[1, 0, Ts, 0]           Bd = [[1/2*Ts^2, 0]
 * (n*n)  [0, 1, 0, Ts]          (n*m)  [0, 1/2*Ts^2]
 *        [0, 0, 1,  0]                 [Ts,       0]
 *        [0, 0, 0,  1]]                [0,       Ts]]
 *  n = 4, m = 2
*/

namespace mpc_controller
{
    MPC_Model::MPC_Model(double dt, int horizon)
     : dt_(dt), N_(horizon)    // 将传入的采样时间和预测步长存入成员变量dt和N_中
    {
        // 双积分器离散化
        Ad_.setIdentity();
        Ad_(0, 2) = dt_;
        Ad_(1, 3) = dt_;

        Bd_.setZero();
        Bd_(0, 0) = 0.5 * dt_ * dt_;
        Bd_(1, 1) = 0.5 * dt_ * dt_;
        Bd_(2, 0) = dt_;
        Bd_(3, 1) = dt_; 

        // 堆叠预测矩阵Phi(Nn * n)
        Phi_.resize(kStateDim * N_, kStateDim);  // 分配4N*4的内存
        AType Ak = AType::Identity(); 
        for(int i = 0; i < N_; ++i) {
            Ak = Ak * Ad_;
            Phi_.block<kStateDim, kStateDim>(kStateDim * i, 0) = Ak;   // 将算好的Ad^[i+1]塞进Phi的第i块行
        }

        // 构建Gamma矩阵(Nn * Nm)
        Gamma_.resize(kStateDim * N_, kInputDim * N_);
        Gamma_.setZero();
        for (int i = 0; i < N_; ++i) {
            AType Aij = AType::Identity();
            for(int j = i; j >= 0; --j) {
                Gamma_.block<kStateDim, kInputDim>(kStateDim  * i, kInputDim * j) = Aij * Bd_;
                Aij = Aij * Ad_;
            }
        }
    }
}