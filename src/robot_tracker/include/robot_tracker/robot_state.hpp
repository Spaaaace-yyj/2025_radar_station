#ifndef ROBOT_STATE_HPP_
#define ROBOT_STATE_HPP_

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <vector>

#include "kalman_fliter.hpp"

class Robot{
public:
    Robot() = default;
    ~Robot() = default;

    enum State{
        TRACKING,
        PREDICTING,
        LOST,
    };

    enum InputState{
        TRACKING_INPUT,
        LOST_INPUT,
    };

    enum RobotID{
        R1 = 0,
        R2,
        R3,
        R4,
        R7,
        B1,
        B2,
        B3,
        B4,
        B7
    };

public:
    void init(float dt, RobotID id);

    void init_state(cv::Point3f measurement_position);

    void init_ekf();

    void update_ekf();

    void update_state(float dt, InputState input_state);

    void track();

    void lost();

    void predict_position();

    void lost_target();

private:


public:
    float width_ = 0.0;// x-x
    float height_ = 0.0;// y-y
    float depth_ = 0.0;// z-z

    cv::Point3f measurement_position_;
    cv::Point3f predict_position_;
    cv::Point3f predict_velocity_;
    State state_ = State::LOST;
    RobotID id_ = RobotID::R1;

    int lost_count_ = 0;
    int max_lost_count_ = 100;

private:
    float dt_ = 0.2;
    EKF *ekf_ = nullptr;


};

#endif 