#include "../include/radar_station/robot.hpp"

void Robot::get_real_pos(){
    for(size_t i = 0; i < robot_points_roi_.size(); i++){
        real_pos_.x += robot_points_roi_[i].x;
        real_pos_.y += robot_points_roi_[i].y;
        real_pos_.z += robot_points_roi_[i].z;
    }
    real_pos_.x /= robot_points_roi_.size();
    real_pos_.y /= robot_points_roi_.size();
    real_pos_.z /= robot_points_roi_.size();
}