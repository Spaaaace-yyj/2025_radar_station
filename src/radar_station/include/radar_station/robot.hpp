#ifndef ROBOT_HPP_ 
#define ROBOT_HPP_

#include <opencv4/opencv2/opencv.hpp>

class Robot{
public:
    Robot() = default;
    ~Robot() = default;

    void get_real_pos();

public:
    int id_ = -1;
    std::vector<cv::Point3f> robot_points_roi_;
    cv::Point3f real_pos_;

};




#endif