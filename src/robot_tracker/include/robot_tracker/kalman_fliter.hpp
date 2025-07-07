#ifndef KALMAN_FLITER_HPP_
#define KALMAN_FLITER_HPP_

#include <Eigen/Dense>
#include <iostream>
#include <cmath>

#include <Eigen/Dense>
#include <cmath>

class EKF {
public:
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix3x6d = Eigen::Matrix<double, 3, 6>;

    EKF(double dt);

    void init(const Vector6d& x0, const Matrix6d& P0);
    void predict();
    void update(const Eigen::Vector3d& z);

    const Vector6d& getState() const { return x_; }

private:
    double dt_;           // time step

    Vector6d x_;          // state vector [x, y, z, vx, vy, vz]
    Matrix6d P_;          // covariance matrix

    Matrix6d Q_;          // process noise
    Eigen::Matrix3d R_;   // observation noise
    Matrix3x6d H_;        // observation matrix
};


#endif  // KALMAN_FILTER_HPP_
