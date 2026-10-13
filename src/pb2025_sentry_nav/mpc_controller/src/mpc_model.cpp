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
        // 单积分器: x_{k+1} = x_k + dt * u_k
        Ad_.setIdentity();                       // 2x2 单位阵
        Bd_.setZero();
        Bd_ = dt_ * Eigen::Matrix2d::Identity(); // 2x2

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