#include "../include/robot_tracker/kalman_fliter.hpp"

EKF::EKF(double alpha_q, double alpha_r, int state_size, int meas_size)
    : state_size_(state_size), meas_size_(meas_size), alpha_q_(alpha_q), alpha_r_(alpha_r)
{
    x_ = Eigen::VectorXd::Zero(state_size_);
    P_ = Eigen::MatrixXd::Identity(state_size_, state_size_);
    Q_ = alpha_q_ * Eigen::MatrixXd::Identity(state_size_, state_size_);
    R_ = alpha_r_ * Eigen::MatrixXd::Identity(meas_size_, meas_size_);
    H_ = Eigen::MatrixXd::Zero(meas_size_, state_size_);

    for (int i = 0; i < meas_size_; ++i)
        H_(i, i) = 1.0;
}

Eigen::VectorXd EKF::f(const Eigen::VectorXd& x, double dt) const {
    Eigen::VectorXd x_pred = x;
    for (int i = 0; i < 3; ++i) {
        double pos = x(i);
        double vel = x(i + 3);
        x_pred(i)     = pos + vel * dt + 0.01 * std::sin(pos);
        x_pred(i + 3) = vel + 0.005 * std::sin(pos);
    }
    return x_pred;
}

Eigen::MatrixXd EKF::computeJacobianF(const Eigen::VectorXd& x, double dt) const {
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(state_size_, state_size_);
    for (int i = 0; i < 3; ++i) {
        double pos = x(i);
        F(i, i + 3) = dt;
        F(i, i)     += 0.01 * std::cos(pos);
        F(i + 3, i) = 0.005 * std::cos(pos);
    }
    return F;
}

void EKF::update(const Eigen::VectorXd& measurement, double dt) {
    // 预测
    Eigen::VectorXd x_pred = f(x_, dt);
    Eigen::MatrixXd F = computeJacobianF(x_, dt);
    Eigen::MatrixXd P_pred = F * P_ * F.transpose() + Q_;

    // 更新
    Eigen::VectorXd z = measurement;
    Eigen::VectorXd y = z - H_ * x_pred;
    Eigen::MatrixXd S = H_ * P_pred * H_.transpose() + R_;
    Eigen::MatrixXd K = P_pred * H_.transpose() * S.inverse();

    x_ = x_pred + K * y;
    P_ = (Eigen::MatrixXd::Identity(state_size_, state_size_) - K * H_) * P_pred;
}

Eigen::Vector3d EKF::getCurrentPosition() const {
    return x_.segment<3>(0);
}

Eigen::Vector3d EKF::predictFuturePosition(double extrapolate_dt, double scale) const {
    Eigen::VectorXd x_future = f(x_, extrapolate_dt);
    x_future *= scale;
    return x_future.segment<3>(0);
}

void EKF::reset() {
    x_ = Eigen::VectorXd::Zero(state_size_);
    P_ = Eigen::MatrixXd::Identity(state_size_, state_size_);
}

Eigen::VectorXd EKF::getCurrentState() const {
    return x_;
}

Eigen::VectorXd EKF::getFutureState(double extrapolate_dt, double scale) const {
    Eigen::VectorXd x_future = f(x_, extrapolate_dt);
    x_future *= scale;
    return x_future;
}
