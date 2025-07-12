#ifndef ONNX_TEST_ONNX_TEST_HPP_
#define ONNX_TEST_ONNX_TEST_HPP_

//opencv
#include <opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/dnn.hpp>

//ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include <cv_bridge/cv_bridge.h>

#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/transform_stamped.hpp>


//math
#include <Eigen/Geometry>

//c++
#include <iostream>
#include <vector>

class OnnxTest : public rclcpp::Node
{
public:
    OnnxTest();
    ~OnnxTest() = default;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void load_model();

    void process_tensor(cv::Mat& outs, cv::Mat& frame);

private:
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;

    std::string car_onnx_path = "/home/spaaaaace/Code/mid70/2025_radar_station/models/car.onnx";
    std::string armor_onnx_path = "/home/spaaaaace/Code/mid70/2025_radar_station/models/armor.onnx";

    cv::dnn::Net car_net;
    cv::dnn::Net armor_net;

};


#endif