#ifndef KALMAN_FLITER_HPP_
#define KALMAN_FLITER_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <cmath>

class EKF {
public:
    EKF(double alpha_q = 1e-5, double alpha_r = 1e-2, int state_size = 6, int meas_size = 3);

    void update(const Eigen::VectorXd& measurement, double dt);
    Eigen::Vector3d getCurrentPosition() const;
    Eigen::Vector3d predictFuturePosition(double extrapolate_dt, double scale) const;
    Eigen::VectorXd getCurrentState() const;
    Eigen::VectorXd getFutureState(double extrapolate_dt, double scale) const;
    void reset();

private:

    Eigen::VectorXd f(const Eigen::VectorXd& x, double dt) const;
    Eigen::MatrixXd computeJacobianF(const Eigen::VectorXd& x, double dt) const;

private:
    int state_size_;
    int meas_size_;

    double alpha_q_;
    double alpha_r_;

    Eigen::VectorXd x_;  // 状态向量
    Eigen::MatrixXd P_;  // 状态协方差矩阵
    Eigen::MatrixXd Q_;  // 过程噪声协方差矩阵
    Eigen::MatrixXd R_;  // 观测噪声协方差矩阵
    Eigen::MatrixXd H_;  // 观测矩阵
};

#endif  // KALMAN_FILTER_HPP_
