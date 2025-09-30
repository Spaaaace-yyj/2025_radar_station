#include "../include/robot_tracker/kalman_fliter.hpp"

EKF::EKF(double dt) : dt_(dt) {
    // Initialize observation matrix H
    H_.setZero();
    H_(0, 0) = 1;
    H_(1, 1) = 1;
    H_(2, 2) = 1;

    // Set process noise Q
    Q_.setIdentity();
    Q_ *= 3e-2; // You can tune this

    // Set observation noise R
    R_.setIdentity();
    R_ *= 1e-1; // You can tune this
}

void EKF::init(const Vector6d& x0, const Matrix6d& P0) {
    x_ = x0;
    P_ = P0;
}

void EKF::predict() {
    // State transition
    Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
    F(0, 3) = dt_;
    F(1, 4) = dt_;
    F(2, 5) = dt_;

    // Predict state
    x_ = F * x_;

    // Predict covariance
    P_ = F * P_ * F.transpose() + Q_;
}

void EKF::update(const Eigen::Vector3d& z) {
    Eigen::Vector3d y = z - H_ * x_;  // Innovation
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_; // Innovation covariance
    Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse(); // Kalman gain

    x_ = x_ + K * y;
    P_ = (Matrix6d::Identity() - K * H_) * P_;
}