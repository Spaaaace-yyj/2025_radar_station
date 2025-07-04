#ifndef RADAR_STATION_HPP_
#define RADAR_STATION_HPP_

//opencv
#include <opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/dnn.hpp>

#include "openvino/openvino.hpp"

//ros2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
#include <cv_bridge/cv_bridge.h>

//math
#include <Eigen/Geometry>

//user
#include "onnx_box.hpp"
#include "robot.hpp"

//c++
#include <iostream>
#include <vector>


class RadarStation : public rclcpp::Node
{
public:
    RadarStation();
private:

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    void generate_color_map();

    void publish_image();
    void publish_onnx_debug_image();
    void publish_cloud();

    std::vector<OnnxBox> get_armor_box(const cv::Mat& src);

    void load_onnx_model();

    std::vector<OnnxBox> process_onnx_result(const cv::Mat& onnx_result, cv::Mat src);

    float get_iou(OnnxBox box1, OnnxBox box2);

    void nms(std::vector<OnnxBox>& boxes, float iou_threshold);

private:
    //ros topic
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    //ros publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr onnx_debug_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
private:
    //save cloud
    int save_cloud_and_image_ = 0;
    int save_count = 0;
    bool save_image_only = 0;
    std::string file_path = "/home/spaaaaace/Code/mid70/2025_radar_station/image_cloud";
    
    float range_of_roi_ = 0.35f;

    //onnx
    std::string car_model_path_ = "/home/spaaaaace/Code/mid70/2025_radar_station/models/car.onnx";
    std::string armor_model_path_ = "/home/spaaaaace/Code/mid70/2025_radar_station/models/armor.onnx";

    cv::dnn::Net car_net;
    cv::dnn::Net armor_net;

    float car_confidence_threshold_ = 0.5f;
    float armor_confidence_threshold_ = 0.5f;

    //lidar_sum_frame_num
    int lidar_frame_add_num_ = 3;

    //stastic_point_fix
    float T_add_x_ = 0.0f;
    float T_add_y_ = 0.0f;
    float T_add_z_ = 0.0f;

    //debug frame
    cv::Mat raw_frame_;
    cv::Mat onnx_debug_frame_;
    cv::Mat frame_;
    std::vector<cv::Scalar> colormap_;

    int lidar_frame_counter_ = 0;

    double dt = 1.0f;
    int64_t start_image_time_ = 0;
    int64_t end_image_time_ = 0;

    //if your image is rotated, you should set this to true
    bool flip_image_ = true;
    
    //point cloud    
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_pub_ = std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    //camera intrinsic
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    	1635.80929422889, 0, 709.797419508020,
    	0, 1636.89792281429, 533.441903861457,
    	0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
    	-0.105728588814658, 0.642952312620478, 0.000000, 0.000000, 0.000000);

    Eigen::Matrix3f R_world_camera_;


    Eigen::Vector3f T_world_camera_;

    int image_width = 1280;
    int image_height = 1024;

    //robot
    std::vector<Robot> target_;

};


#endif //RADAR_STATION_HPP_
