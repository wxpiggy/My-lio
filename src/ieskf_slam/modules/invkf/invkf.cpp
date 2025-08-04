#include "ieskf_slam/modules/invkf/invkf.h"
#include "ieskf_slam/math/SEn3.h"
#include "ieskf_slam/math/SE3.h"
#include "ieskf_slam/math/SO3.h"
#include <Eigen/src/Core/Matrix.h>
using namespace liepp;
namespace IESKFSlam{
    INVKF::INVKF(const std::string & config_path,const std::string &prefix):ModuleBase(config_path,prefix,"INVKF"){
        X_ = Eigen::MatrixXd::Identity(5,5);
        theta_ = Eigen::MatrixXd::Zero(6,1);
        P_ = Eigen::MatrixXd::Identity(15,15);
        //Q = Eigen::MatrixXd::Zero(15,15);
        double cov_gyroscope,cov_acceleration,cov_bias_acceleration,cov_bias_gyroscope;
        P_(9,9)   = P_(10,10) = P_(11,11) = 0.0001;
        P_(12,12) = P_(13,13) = P_(14,14) = 0.001;
        P_(15,15) = P_(16,16) = P_(17,17) = 0.00001; 
        readParam("cov_gyroscope",cov_gyroscope,0.1);
        readParam("cov_acceleration",cov_acceleration,0.1);
        readParam("cov_bias_acceleration",cov_bias_acceleration,0.1);
        readParam("cov_bias_gyroscope",cov_bias_gyroscope,0.1);
        Q = Eigen::MatrixXd::Zero(15,15);
        Q.block<3, 3>(0, 0).diagonal() = Eigen::Vector3d{cov_gyroscope,cov_gyroscope,cov_gyroscope};
        Q.block<3, 3>(3, 3).diagonal() = Eigen::Vector3d{cov_acceleration,cov_acceleration,cov_acceleration};
        Q.block<3, 3>(9, 9).diagonal() = Eigen::Vector3d{cov_bias_gyroscope,cov_bias_gyroscope,cov_bias_gyroscope};
        Q.block<3, 3>(12, 12).diagonal() = Eigen::Vector3d{cov_bias_acceleration,cov_bias_acceleration,cov_bias_acceleration};
        gravity.setZero();
    }
    INVKF::~INVKF(){

    }
    void INVKF::predict(IMU &imu, double dt){
        Eigen::Vector3d gyroscope = imu.gyroscope - theta_.head(3);
        Eigen::Vector3d acceleration = imu.acceleration - theta_.tail(3);
        
        Eigen::Matrix3d rotation = X_.block(0,0,3,3);
        Eigen::Vector3d velocity = X_.block(0,3,3,1);
        Eigen::Vector3d position = X_.block(0,4,3,1);

        Eigen::Vector3d phi = gyroscope * dt;
        Eigen::Matrix3d rotation_pred = rotation * SO3<double>::exp(phi).asMatrix();
        Eigen::Vector3d velocity_pred = velocity + (rotation * acceleration + gravity) * dt;
        Eigen::Vector3d position_pred = position + velocity * dt + 0.5 * (rotation * acceleration + gravity) * dt * dt;
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(15,15);
        setRotation(rotation_pred);
        setVelocity(velocity_pred);
        setPosition(position_pred);
        A.block(3,0,3,3) = SO3<double>::skew(gravity) * dt;
        A.block(6,3,3,3) = Eigen::Matrix3d::Identity();
        A.block(0,9,3,3) = rotation * (-1) * dt;
        A.block(3,9,3,3) = -1 * SO3<double>::skew(velocity) * rotation * dt;
        A.block(3,12,3,3) = -1 * rotation * dt;
        A.block(6,9,3,3) = -1 * SO3<double>::skew(position) * rotation * dt;

        Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(15,15) + A;
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Identity(15,15);
        
        Adj.block(0,0,9,9) = SEn3<2,double>::Adjoint_SEK3(X_);//!!!!!!!!!!!!!!!!11
        
        Eigen::MatrixXd PhiAdj = Phi * Adj;//std::cout << "5"  << std::endl;
        Eigen::MatrixXd Q_hat = PhiAdj * Q * PhiAdj.transpose() * dt;
        P_ = Phi * P_ * Phi.transpose() + Q_hat;
    }
    bool INVKF::update(){

    //     Eigen::Matrix <double,5,5> x = X_;
    //     Eigen::MatrixXd K;
    //     Eigen::MatrixXd H;
    //     Eigen::MatrixXd z;

    //     calc_zh_ptr->calculate(x,z,H);
    //     Eigen::MatrixXd H_t = H.transpose();
    //     K = (H_t*H+(P_/0.001).inverse()).inverse()*H_t; //公式18
    //     Eigen::VectorXd delta = K * z ;
    //     Eigen::MatrixXd dX = SEn3<2,double>::Exp_SEK3(delta.segment(0,9));//!!!!!!!!!
    //     Eigen::VectorXd dTheta = delta.segment(9, 6);
    //    // Eigen::Vector3d dGravity = delta.segment(15,3);
    //     setX(dX*X_);
    //     setTheta(theta_ + dTheta);
    //    // setGravity(gravity + dGravity);
    //     std::cout <<"gravity : " << gravity[0] <<"," << gravity[1] <<","<< gravity[2] << std::endl;
    //     // auto x_k_k = X_;

    //     // auto x_1 = getErrorState(x_k_k,X_);
    //     // calc_zh_ptr->calculate(x_k_k,z_k,H_k);
    //     // Eigen::MatrixXd H_kt = H_k.transpose();
    //     // // R 直接写死0.001; 
    //     // K = (H_t*H+(P_/0.001).inverse()).inverse()*H_t; //公式18
    //     // Eigen::VectorXd delta =   K * z ;
    //     // Eigen::MatrixXd dX = Exp_SEK3(delta.segment(0,9));
    //     // Eigen::VectorXd dTheta = delta.segment(9, 6);
    //     // setX(dX*X_);
    //     // setTheta(theta_ + dTheta);
        
    //     P_ = (Eigen::Matrix<double,15,15>::Identity()-K*H)*P_;

        
    //     return true;
        static int cnt_ = 0;
        Eigen::Matrix<double, 5,5 > x_k_k = X_;
        Eigen::Matrix<double, 5,5 >  x_k_last = X_;
        Eigen::VectorXd theta_k_k = theta_;
        Eigen::VectorXd theta_k_last = theta_;
        Eigen::MatrixXd K;
        Eigen::MatrixXd H_k;
        Eigen::Matrix<double,15,15> P_in_update;
        bool converge =true;
        
        for (int i = 0; i < 10; i++){
            SEn3<2,double> err = SEn3<2,double>::Identity();
            err.fromMatrix(getErrorState(x_k_k, X_));
            Eigen::Matrix<double, 5,5 > err_state_eigen = err.asMatrix();
            // SEn3<2,double>::leftJacobian(SEn3<2,double>::log(err));
            Eigen::Matrix<double,15,15> J_inv = Eigen::Matrix<double,15,15>::Identity();
            J_inv.block<9,9>(0,0) = SEn3<2,double>::leftJacobian(SEn3<2,double>::log(err));
            J_inv.block<6,6>(9,9) = Eigen::Matrix<double, 6, 6>::Identity();
            Eigen::MatrixXd z_k;
            Eigen::MatrixXd R_inv;
            calc_zh_ptr->calculate(x_k_k,z_k,H_k);
            Eigen::MatrixXd H_kt = H_k.transpose();
             P_in_update = J_inv* P_ *J_inv.transpose();
            // R 直接写死0.001; 
            K = (H_kt*H_k+(P_in_update/0.001).inverse()).inverse()*H_kt; //公式18
            // K = P_in_update * H_kt * (H_k * P_in_update * H_kt + R).inverse();
            //. 计算X 的增量
            Eigen::VectorXd err_full;
            err_full.resize(15);
            err_full.segment(0, 9) = SEn3<2,double>::vee(err_state_eigen);
            err_full.segment(9,6) = theta_k_k-theta_;
           // std::cout << err_full.segment(9,6) << std::endl;
            Eigen::MatrixXd left = K*z_k;
            Eigen::MatrixXd right = (Eigen::Matrix<double,15,15>::Identity()-K*H_k)*J_inv * err_full ;
            Eigen::VectorXd update_x = left +right ;//误差的更新
            converge =true;
            for ( int idx = 0; idx < 15; idx++)
            {
                if (std::abs(update_x(idx,0))>0.001)
                {
                    converge = false;
                    break;
                }
                
            }
            x_k_k =   SEn3<2,double>::Exp_SEK3(update_x.segment(0,9))*  x_k_k ;
            theta_k_k = theta_k_k + update_x.segment(9,6);
            if(converge){
                std::cout << i << std::endl;
                break;
            }
            //  std::cout << "update_x   " << update_x.size();
        }
        cnt_++;
        setX(x_k_k);
        setTheta(theta_k_k);
        P_ = (Eigen::Matrix<double,15,15>::Identity()-K*H_k)*P_in_update;
        
       return converge;
 
    }

    const Eigen::MatrixXd INVKF::getX(){return X_;}
    const Eigen::VectorXd INVKF::getTheta(){return theta_;}
    const Eigen::MatrixXd INVKF::getP(){return P_;}
    const Eigen::Matrix3d INVKF::getRotation(){return X_.block(0,0,3,3);}
    const Eigen::Vector3d INVKF::getVelocity(){return X_.block(0,3,3,1);}
    const Eigen::Vector3d INVKF::getPosition(){return X_.block(0,4,3,1);}
    const Eigen::Vector3d INVKF::getGyroscopeBias(){return theta_.head(3);}
    const Eigen::Vector3d INVKF::getAccelerometerBias(){return theta_.tail(3);}
    const Eigen::Vector3d INVKF::getGravity(){return gravity;}
    void INVKF::setX(const Eigen::MatrixXd& X){X_ = X;}
    void INVKF::setP(const Eigen::MatrixXd& P){P_ = P;}
    void INVKF::setTheta(const Eigen::VectorXd& theta){theta_ = theta;}
    void INVKF::setRotation(const Eigen::Matrix3d& R){X_.block(0,0,3,3) = R;}
    void INVKF::setVelocity(const Eigen::Vector3d& v){X_.block(0,3,3,1) = v;}
    void INVKF::setPosition(const Eigen::Vector3d& p){X_.block(0,4,3,1) = p;}
    void INVKF::setGyroscopeBias(const Eigen::Vector3d& bg){theta_.head(3) = bg;}
    void INVKF::setAccelerometerBias(const Eigen::Vector3d& ba){theta_.tail(3) = ba;}
    void INVKF::setGravity(const Eigen::Vector3d& g){gravity = g;}
    Eigen::VectorXd INVKF::getErrorTheta(const Eigen::VectorXd &s1, const Eigen::VectorXd &s2){return s1-s2;}
    Eigen::MatrixXd INVKF::getErrorState(const Eigen::MatrixXd &s1, const Eigen::MatrixXd &s2){
        return s1*s2.inverse();
    }
}