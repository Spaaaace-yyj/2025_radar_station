#ifndef RADAR_WORLD_CALIB_HPP_
#define RADAR_WORLD_CALIB_HPP_

//opencv
#include <opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/dnn.hpp>

//ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>

//math
#include <Eigen/Geometry>

//c++
#include <iostream>
#include <vector>

static bool left_button_is_down_;

static int point_count = 1;

static cv::Point2d mouse_point_;

static std::vector<cv::Point2d> image_points_;

static std::vector<cv::Point3f> world_points_ = {
    cv::Point3f(0, 0, 0),
    cv::Point3f(0.6, 0, 0),
    cv::Point3f(0.6, 0.6, 0),
    cv::Point3f(0, 0.6, 0),
};

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    	1635.80929422889, 0, 709.797419508020,
    	0, 1636.89792281429, 533.441903861457,
    	0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
    -0.105728588814658, 0.642952312620478, 0.000000, 0.000000, 0.000000);

cv::Mat rvec, tvec;

class RadarWorldCalib : public rclcpp::Node
{
public:
    RadarWorldCalib();
    ~RadarWorldCalib() = default;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    static void onMouse(int event, int x, int y, int flags, void* userdata);

    void draw_x(cv::Mat& image, cv::Point2d point);

    void draw_point(cv::Mat& image, cv::Point2d point, int point_id);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;



    bool flip_image = true;

};



#endif