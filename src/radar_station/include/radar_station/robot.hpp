#ifndef ROBOT_HPP_ 
#define ROBOT_HPP_

#include <opencv4/opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <iostream>

class Robot{
public:
    Robot() = default;
    ~Robot() = default;

    void get_real_pos();

    void get_world_location(Eigen::Matrix3f& R_world_camera_, Eigen::Vector3f& T_world_camera_);

public:

    std::vector<cv::Point3f> robot_rect_3d_;

    int id_ = -1;
    std::vector<cv::Point3f> robot_points_roi_;
    cv::Point3f real_pos_;
    cv::Point3f world_pos_;

    float dis_to_zero_ = 0.0;

    float width_ = 0.0;
    float height_ = 0.0;
    float depth_ = 0.0;

};




#endif