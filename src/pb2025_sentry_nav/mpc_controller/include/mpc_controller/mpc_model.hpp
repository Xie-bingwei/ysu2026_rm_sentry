#ifndef MPC_CONTROLLER__MPC_MODEL_HPP_
#define MPC_CONTROLLER__MPC_MODEL_HPP_

#include <Eigen/Dense>

namespace mpc_controller
{
    class MPC_Model
    {
        public:
        static constexpr int kStateDim = 4;     // s=[px, pv, vx, vy]
        static constexpr int kInputDim = 2;     // u=[ax, ay]

        using AType = Eigen::Matrix<double, kStateDim, kStateDim>;
        using BType = Eigen::Matrix<double, kStateDim, kInputDim>;

        // dt->Ts, horison->N
        MPC_Model(double dt, int horizon);

        // 以下参数都为常量，故不需要修改类的任何成员变量(在函数后加const)
        const AType& Ad() const {return Ad_;}                  
        const BType& Bd() const {return Bd_;}
        const Eigen::MatrixXd& Phi() const {return Phi_;}
        const Eigen::MatrixXd& Gamma() const {return Gamma_;}

        int horizon() const {return N_;}
        double dt() const {return dt_;}

        private:
        double dt_;
        int N_;
        AType Ad_;
        BType Bd_;
        Eigen::MatrixXd Phi_;     // 其维度取决于N,每个元素为double,配置动态大小矩阵
        Eigen::MatrixXd Gamma_;
    };
}

#endif