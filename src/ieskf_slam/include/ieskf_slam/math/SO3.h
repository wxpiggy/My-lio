#pragma once
#include <Eigen/Dense>
namespace IESKFSlam {
    static inline Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d &so3) {
        Eigen::Matrix3d so3_skew_sym;
        so3_skew_sym.setZero();
        so3_skew_sym(0, 1) = -1 * so3(2);
        so3_skew_sym(0, 2) = so3(1);
        so3_skew_sym(1, 2) = -1 * so3(0);
        so3_skew_sym(1, 0) = so3(2);
        so3_skew_sym(2, 0) = -1 * so3(1);
        so3_skew_sym(2, 1) = so3(0);
        return so3_skew_sym;
    }
    static Eigen::Matrix3d so3Exp(const Eigen::Vector3d &so3) {
        Eigen::Matrix3d SO3;
        double so3_norm = so3.norm();
        if (so3_norm <= 1e-7) {
            SO3.setIdentity();
            return SO3;
        }
        Eigen::Matrix3d so3_skew_sym = skewSymmetric(so3);
        SO3 = Eigen::Matrix3d::Identity() + (so3_skew_sym / so3_norm) * sin(so3_norm) +
              (so3_skew_sym * so3_skew_sym / (so3_norm * so3_norm)) * (1 - cos(so3_norm));

        return SO3;
    }
    static Eigen::Vector3d SO3Log(const Eigen::Matrix3d &SO3) {
        double theta = (SO3.trace() > 3 - 1e6) ? 0 : acos((SO3.trace() - 1) / 2);
        Eigen::Vector3d so3(SO3(2, 1) - SO3(1, 2), SO3(0, 2) - SO3(2, 0), SO3(1, 0) - SO3(0, 1));
        return fabs(theta) < 0.001 ? (0.5 * so3) : (0.5 * theta / sin(theta) * so3);
    }
    static Eigen::Matrix3d A_T(const Eigen::Vector3d &v) {
        Eigen::Matrix3d res;
        double squaredNorm = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
        double norm = std::sqrt(squaredNorm);
        if (norm < 1e-11) {
            res = Eigen::Matrix3d::Identity();
        } else {
            res = Eigen::Matrix3d::Identity() + (1 - std::cos(norm)) / squaredNorm * skewSymmetric(v) +
                  (1 - std::sin(norm) / norm) / squaredNorm * skewSymmetric(v) * skewSymmetric(v);
        }
        return res;
    }
    static Eigen::MatrixXd Adjoint_SEK3(const Eigen::MatrixXd X) {
        int K = X.cols() - 3;
        Eigen::MatrixXd Adj = Eigen::MatrixXd::Zero(3 + 3 * K, 3 + 3 * K);
        Eigen::Matrix3d R = X.block(0, 0, 3, 3);
        Adj.block(0, 0, 3, 3) = R;
        for (int i = 0; i < K; i++) {
            Adj.block(3 + 3 * i, 3 + 3 * i, 3, 3) = R;
            Adj.block(3 + 3 * i, 0, 3, 3) = skewSymmetric(X.block(0, 3 + i, 3, 1)) * R;
        }
        return Adj;
    }
    static Eigen::MatrixXd Exp_SEK3(const Eigen::VectorXd &v) {
        // Computes the vectorized exponential map for SE_K(3)
        int K = (v.size() - 3) / 3;
        Eigen::MatrixXd X = Eigen::MatrixXd::Identity(3 + K, 3 + K);
        Eigen::Matrix3d R;
        Eigen::Matrix3d Jl;
        Eigen::Vector3d w = v.head(3);
        double theta = w.norm();
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        if (theta < 1e-11) {
            R = I;
            Jl = I;
        } else {
            Eigen::Matrix3d A = skewSymmetric(w);
            double theta2 = theta * theta;
            double stheta = sin(theta);
            double ctheta = cos(theta);
            double oneMinusCosTheta2 = (1 - ctheta) / (theta2);
            Eigen::Matrix3d A2 = A * A;
            R = I + (stheta / theta) * A + oneMinusCosTheta2 * A2;
            Jl = I + oneMinusCosTheta2 * A + ((theta - stheta) / (theta2 * theta)) * A2;
        }
        X.block<3, 3>(0, 0) = R;
        for (int i = 0; i < K; ++i) {
            X.block<3, 1>(0, 3 + i) = Jl * v.segment<3>(3 + 3 * i);
        }
        return X;
    }

}  // namespace IESKFSlam