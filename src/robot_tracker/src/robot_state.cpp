#include "../include/robot_tracker/robot_state.hpp"

void Robot::init(float dt, RobotID id){
    dt_ = dt;
    id_ = id;
    ekf_ = new EKF(dt_);
}

void Robot::init_ekf(){
    EKF::Vector6d x0;
    x0 << measurement_position_.x, measurement_position_.y, measurement_position_.z, 0, 0, 0;
    EKF::Matrix6d P0 = EKF::Matrix6d::Identity() * 0.1;
    ekf_->init(x0, P0);
}

void Robot::update_ekf(){
    ekf_->predict();
    Eigen::Vector3d z;
    z << measurement_position_.x, measurement_position_.y, measurement_position_.z;
    ekf_->update(z);
    EKF::Vector6d x = ekf_->getState();
    predict_position_.x = x(0);
    predict_position_.y = x(1);
    predict_position_.z = x(2);
    predict_velocity_.x = x(3);
    predict_velocity_.y = x(4);
    predict_velocity_.z = x(5);
}

void Robot::update_state(float dt, InputState input_state){
    dt_ = dt;
    switch (input_state)
    {
    case InputState::TRACKING_INPUT:
        track();
        break;
    case InputState::LOST_INPUT:
        lost();
        break;
    default:
        break;
    }
}

void Robot::track(){
    if(state_ == State::LOST){
        init_ekf();
    }
    state_ = State::TRACKING;
    lost_count_ = 0;
    update_ekf();
}

void Robot::lost(){
    if(state_ == State::TRACKING || state_ == State::PREDICTING){
        if(lost_count_ <= max_lost_count_){
            state_ = State::PREDICTING;
            lost_count_++;
            predict_position();
        }else{
            state_ = State::LOST;
        }
    }
}

void Robot::predict_position(){
    predict_position_ = predict_position_ + predict_velocity_ * dt_;

}



// Robot::~Robot(){
//     if(ekf_ != nullptr){
//         delete ekf_;
//         ekf_ = nullptr;
//     }
// }
