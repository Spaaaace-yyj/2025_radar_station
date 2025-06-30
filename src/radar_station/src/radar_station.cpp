//opencv
#include <opencv4/opencv2/opencv.hpp>
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
#include "../include/radar_station/radar_station.hpp"
//c++
#include <iostream>
#include <vector>

using namespace std;

RadarStation::RadarStation() : Node("radar_station")
{
    this->declare_parameter("save_cloud_and_image", 0);

    this->get_parameter("save_cloud_and_image", save_cloud_and_image_);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/lidar", 10, std::bind(&RadarStation::point_cloud_callback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&RadarStation::image_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/radar_station/image_debug", 10);

    generate_color_map();

    RCLCPP_INFO(this->get_logger(), "Radar station node has been created");
}
void RadarStation::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    this->get_parameter("save_cloud_and_image", save_cloud_and_image_);

    if (!point_cloud_ || point_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Point cloud is empty or null");
        return;
    }
    if(lidar_frame_counter_ >= lidar_frame_add_num_){
        lidar_frame_counter_ = 0;
        start_image_time_ = cv::getTickCount();
        cv::Mat image_mat_(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
        
        if(flip_image_){
            cv::rotate(image_mat_, frame_, cv::ROTATE_180);
        }else{
            frame_ = image_mat_.clone();
        }
        cv::cvtColor(frame_, frame_, cv::COLOR_BGR2RGB);
        if(save_cloud_and_image_ == 1){
            std::string cloud_filename = file_path + "/" + std::to_string(save_count) + ".pcd";
            std::string image_filename = file_path + "/" + std::to_string(save_count) + ".jpg";
            pcl::io::savePCDFileBinary(cloud_filename, *point_cloud_);
            cv::imwrite(image_filename, frame_);
            save_count++;
        }
        
        Eigen::Matrix3f R;
        // R << -0.0173, -0.9998, 0.0043,
        //      0.0053, -0.0044, -1.0000,
        //      0.9998, -0.0173, 0.0054;
        R << -0.0173, 0.0053, 0.9998,
             -0.0098, -0.0044, -0.0173,
             0.0043, -1.0000, 0.0054;

        // R << 1.0, 0.0, 0.0,
        //      0.0, 1.0, 0.0,
        //      0.0, 0.0, 1.0;
        //Eigen::Vector3f T(lidar_x_, lidar_y_, lidar_z_);
        Eigen::Vector3f T(0.1098, 0.0189, -0.5067);
        //Eigen::Vector3f T(0.0, 0.0, 0.0);
        // Eigen::Vector3f T(-0.1098, -0.0189, 0.5067);

        std::vector<cv::Point3f> lidar_points;
        std::vector<cv::Point2f> lidar_points_projection;
        std::vector<float> intensity_values;

        for(size_t i = 0; i < point_cloud_->points.size(); i++){
            const auto& p = point_cloud_->points[i];
            intensity_values.push_back(p.intensity);
            //livox雷达坐标系与opencv坐标系定义不同，转换livox坐标系为opencv坐标系
            Eigen::Vector3f lidar_point(-p.y, -p.z, p.x);
            Eigen::Vector3f lidar_point_in_camera = R * lidar_point + T;
            cv::Point3f point_lidar_in_camera(lidar_point_in_camera.x(), lidar_point_in_camera.y(), lidar_point_in_camera.z());

            lidar_points.push_back(point_lidar_in_camera);
        }
        
        cv::projectPoints(lidar_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, lidar_points_projection);
        
        for(size_t i = 0; i < lidar_points_projection.size(); i++){
            cv::Scalar color;
            int index = static_cast<int>(255.0 * intensity_values[i] / 255);
            if(index > 0 && index < 255){
                color = colormap_[index];
            }else{
                color = cv::Scalar(0, 0, 0);
            }

            cv::circle(frame_, lidar_points_projection[i], 1.5, color, -1);
        }

        
        cv::putText(frame_, "Latency: " + to_string(dt) + "ms", cv::Point2f(5, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, 8);
        end_image_time_ = cv::getTickCount();	
	    dt = (end_image_time_ - start_image_time_) * 1000 / cv::getTickFrequency();
    }
    publish_image();
    cout << "save_cloud_and_image_ = " << save_cloud_and_image_ << endl;
}
void RadarStation::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*msg, *cloud);
    if(lidar_frame_counter_ == 0){
        point_cloud_->clear();
        *point_cloud_ = *cloud;
    }else{
        *point_cloud_ += *cloud;
    }
    RCLCPP_INFO(this->get_logger(), "frame_count = %d", lidar_frame_counter_);
    lidar_frame_counter_++;
}

void RadarStation::generate_color_map(){
    colormap_.resize(256);
    for (int i = 0; i < 256; ++i) {
        cv::Mat gray(1, 1, CV_8UC1, cv::Scalar(i));
        cv::Mat color;
        cv::applyColorMap(gray, color, cv::COLORMAP_RAINBOW);
        cv::Vec3b bgr = color.at<cv::Vec3b>(0, 0);
        colormap_[i] = cv::Scalar(bgr[0], bgr[1], bgr[2]);
    }
}

void RadarStation::publish_image(){
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "camera_frame";
    sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
    image_pub_->publish(*msg);
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto radar_station = std::make_shared<RadarStation>();
    rclcpp::spin(radar_station);
    rclcpp::shutdown();
    return 0;
}


