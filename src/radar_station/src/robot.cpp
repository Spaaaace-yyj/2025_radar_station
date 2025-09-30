#include "../include/radar_station/robot.hpp"

void Robot::get_real_pos(){

    for(size_t i = 0; i < robot_points_roi_.size(); i++){
        if(robot_points_roi_.empty()) break;
        real_pos_.x += robot_points_roi_[i].x;
        real_pos_.y += robot_points_roi_[i].y;
        real_pos_.z += robot_points_roi_[i].z;
    }
    real_pos_.x /= robot_points_roi_.size();
    real_pos_.y /= robot_points_roi_.size();
    real_pos_.z /= robot_points_roi_.size();
    dis_to_zero_ = sqrt(real_pos_.x * real_pos_.x + real_pos_.y * real_pos_.y + real_pos_.z * real_pos_.z);

}

void Robot::get_world_location(Eigen::Matrix3f& R_world_camera_, Eigen::Vector3f& T_world_camera_){
    float max_x = 0, min_x = 10000, max_y = 0, min_y = 10000, max_z = 0, min_z = 10000;
    Eigen::Matrix3f R_camera_world = R_world_camera_.transpose();
    Eigen::Vector3f T_camera_world = - R_camera_world * T_world_camera_;
    

    for(size_t i = 0; i < robot_points_roi_.size(); i++){
        Eigen::Vector3f world_point;
        Eigen::Vector3f robot_point_roi(robot_points_roi_[i].x, robot_points_roi_[i].y, robot_points_roi_[i].z);
        world_point = R_camera_world * robot_point_roi + T_camera_world;
        if(world_point.x() > max_x) max_x = world_point.x();
        if(world_point.x() < min_x) min_x = world_point.x();
        if(world_point.y() > max_y) max_y = world_point.y();
        if(world_point.y() < min_y) min_y = world_point.y();
        if(world_point.z() > max_z) max_z = world_point.z();
        if(world_point.z() < min_z) min_z = world_point.z();
    }

    width_ = max_x - min_x;
    height_ = max_y - min_y;
    depth_ = max_z - min_z;

    Eigen::Vector3f world_pos;
    Eigen::Vector3f real_pos(real_pos_.x, real_pos_.y, real_pos_.z);
    world_pos = R_camera_world * real_pos + T_camera_world;
    world_pos_ = cv::Point3f(world_pos.x(), world_pos.y(), world_pos.z());

}
