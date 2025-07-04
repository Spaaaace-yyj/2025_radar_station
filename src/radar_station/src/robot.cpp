#include "../include/radar_station/robot.hpp"

void Robot::get_real_pos(){
    float max_x = 0, min_x = 10000, max_y = 0, min_y = 10000, max_z = 0, min_z = 10000;
    for(size_t i = 0; i < robot_points_roi_.size(); i++){
        real_pos_.x += robot_points_roi_[i].x;
        real_pos_.y += robot_points_roi_[i].y;
        real_pos_.z += robot_points_roi_[i].z;
        if(robot_points_roi_[i].x > max_x) max_x = robot_points_roi_[i].x;
        if(robot_points_roi_[i].x < min_x) min_x = robot_points_roi_[i].x;
        if(robot_points_roi_[i].y > max_y) max_y = robot_points_roi_[i].y;
        if(robot_points_roi_[i].y < min_y) min_y = robot_points_roi_[i].y;
        if(robot_points_roi_[i].z > max_z) max_z = robot_points_roi_[i].z;
        if(robot_points_roi_[i].z < min_z) min_z = robot_points_roi_[i].z;
    }
    real_pos_.x /= robot_points_roi_.size();
    real_pos_.y /= robot_points_roi_.size();
    real_pos_.z /= robot_points_roi_.size();
    dis_to_zero_ = sqrt(real_pos_.x * real_pos_.x + real_pos_.y * real_pos_.y + real_pos_.z * real_pos_.z);
    get_bounding_cube(max_x - min_x, max_y - min_y, max_z - min_z);
}

void Robot::get_bounding_cube(float width, float height, float depth){
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x - 0.5 * width, real_pos_.y - 0.5 * height, real_pos_.z - 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x + 0.5 * width, real_pos_.y - 0.5 * height, real_pos_.z - 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x + 0.5 * width, real_pos_.y + 0.5 * height, real_pos_.z - 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x - 0.5 * width, real_pos_.y + 0.5 * height, real_pos_.z - 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x - 0.5 * width, real_pos_.y - 0.5 * height, real_pos_.z + 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x + 0.5 * width, real_pos_.y + 0.5 * height, real_pos_.z + 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x + 0.5 * width, real_pos_.y - 0.5 * height, real_pos_.z + 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x - 0.5 * width, real_pos_.y + 0.5 * height, real_pos_.z + 0.5 * depth));
    robot_rect_3d_.push_back(cv::Point3f(real_pos_.x - 0.5 * width, real_pos_.y - 0.5 * height, real_pos_.z + 0.5 * depth));
}

void Robot::get_world_location(Eigen::Matrix3f& R_world_camera_, Eigen::Vector3f& T_world_camera_){

    Eigen::Matrix3f R_camera_world = R_world_camera_.transpose();
    Eigen::Vector3f T_camera_world = - R_camera_world * T_world_camera_;
    
    Eigen::Vector3f world_pos;
    Eigen::Vector3f real_pos(real_pos_.x, real_pos_.y, real_pos_.z);
    world_pos = R_camera_world * real_pos + T_camera_world;
    world_pos_ = cv::Point3f(world_pos.x(), world_pos.y(), world_pos.z());

}
