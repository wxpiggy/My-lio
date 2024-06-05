#pragma once
#include <Eigen/Dense>

#include "ieskf_slam/math/SO3.h"
#include "ieskf_slam/modules/module_base.h"
#include "ieskf_slam/type/imu.h"
namespace IESKFSlam {
    class INVKF : private ModuleBase {
    public:
        using Ptr = std::shared_ptr<INVKF>;
        const Eigen::MatrixXd getX();
        const Eigen::VectorXd getTheta();
        const Eigen::MatrixXd getP();
        const Eigen::Matrix3d getRotation();
        const Eigen::Vector3d getVelocity();
        const Eigen::Vector3d getPosition();
        const Eigen::Vector3d getGyroscopeBias();
        const Eigen::Vector3d getAccelerometerBias();
        const Eigen::Vector3d getGravity();
        void setX(const Eigen::MatrixXd& X);
        void setP(const Eigen::MatrixXd& P);
        void setTheta(const Eigen::VectorXd& Theta);
        void setRotation(const Eigen::Matrix3d& R);
        void setVelocity(const Eigen::Vector3d& v);
        void setPosition(const Eigen::Vector3d& p);
        void setGyroscopeBias(const Eigen::Vector3d& bg);
        void setAccelerometerBias(const Eigen::Vector3d& ba);
        void setGravity(const Eigen::Vector3d& g);
        Eigen::MatrixXd getErrorState(const Eigen::MatrixXd &s1, const Eigen::MatrixXd &s2);
        Eigen::MatrixXd getErrorTheta(const Eigen::MatrixXd &s1, const Eigen::MatrixXd &s2);
        INVKF(const std::string &confif_path, const std::string &prefix);
        ~INVKF();
        void predict(IMU &imu, double dt);
        bool update();
        class CalcZHInterface{
            public:
            virtual bool calculate(Eigen::Matrix<double, 5,5 > &state, Eigen::MatrixXd &z, Eigen::MatrixXd &H) = 0;
        };
        std::shared_ptr<CalcZHInterface> calc_zh_ptr;
        
    private:
        Eigen::MatrixXd X_;
        Eigen::VectorXd theta_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd Q;
        Eigen::Vector3d gravity;


    };

}  // namespace IESKFSlam