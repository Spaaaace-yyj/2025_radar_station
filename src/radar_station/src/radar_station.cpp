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

//c++
#include <iostream>
#include <vector>

using namespace std;

class RadarStation : public rclcpp::Node
{
public:
    RadarStation() : Node("radar_station")
    {
        this->declare_parameter("lidar_x", 0.0);
        this->declare_parameter("lidar_y", 0.0);
        this->declare_parameter("lidar_z", 0.0);
        this->get_parameter("lidar_x", lidar_x_);
        this->get_parameter("lidar_y", lidar_y_);
        this->get_parameter("lidar_z", lidar_z_);

        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar", 10, std::bind(&RadarStation::point_cloud_callback, this, std::placeholders::_1));
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&RadarStation::image_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Radar station node has been created");
    }

private:

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!point_cloud_ || point_cloud_->empty()) {
            RCLCPP_WARN(this->get_logger(), "Point cloud is empty or null");
            return;
        }

        if(lidar_frame_counter_ >= 2){
            lidar_frame_counter_ = 0;
            start_image_time_ = cv::getTickCount();
            cv::Mat image_mat_(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
            cv::Mat frame_;
            if(flip_image_){
                cv::rotate(image_mat_, frame_, cv::ROTATE_180);
            }else{
                frame_ = image_mat_.clone();
            }
            cv::cvtColor(frame_, frame_, cv::COLOR_BGR2RGB);

            Eigen::Vector3f lidar_point;
            Eigen::Matrix3f R;
            R << 1.0, 0.0, 0.0,
                 0.0, 1.0, 0.0,
                 0.0, 0.0, 1.0;
            //Eigen::Vector3f T(lidar_x_, lidar_y_, lidar_z_);
            Eigen::Vector3f T(0.0, -0.3, -0.32);
            for(size_t i = 0; i < point_cloud_->points.size(); i++){
                const auto& p = point_cloud_->points[i];
                lidar_point << p.x, p.y, p.z;
                Eigen::Vector3f lidar_camera_point = R * lidar_point + T;

                cv::Point3f lidar_point_3f(lidar_camera_point.x(), lidar_camera_point.y(), lidar_camera_point.z());
                cv::Point3f camera_point_3f(-lidar_point_3f.y, -lidar_point_3f.z, lidar_point_3f.x);

                std::vector<cv::Point2f> image_points;

                cv::projectPoints(std::vector<cv::Point3f>{camera_point_3f}, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, image_points);
                //std::cout << "image_points: " << image_points[0] << std::endl;

                if(image_points[0].x >= 1280 || image_points[0].x <= 0 || image_points[0].y >= 1024 || image_points[0].y <= 0){
                    continue;
                }

                float norm_intensity = std::min(std::max(p.intensity / 255.0f, 0.0f), 1.0f);
                cv::Scalar color(norm_intensity * 255, norm_intensity * 200, (1.0 - norm_intensity) * 255);
                cv::circle(frame_, image_points[0], 1, color, -1);
            }

            cv::putText(frame_, "Latency: " + to_string(dt) + "ms", cv::Point2f(5, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 1.5, 8);

            cv::imshow("image", frame_); 

            end_image_time_ = cv::getTickCount();	
		    dt = (end_image_time_ - start_image_time_) * 1000 / cv::getTickFrequency();
            cv::waitKey(1);
        }
        
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
private:
    float lidar_x_;
    float lidar_y_;
    float lidar_z_;

    int lidar_frame_counter_ = 0;

    double dt = 1.0f;
    int64_t start_image_time_ = 0;
    int64_t end_image_time_ = 0;
    bool flip_image_ = true;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
    	1809.350937, 0, 584.042731,
    	0, 1803.386013, 462.815584,
    	0, 0, 1);
	cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << 
    	-0.038871, 0.354031, -0.009067, -0.006690, 0.000000);

};



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto radar_station = std::make_shared<RadarStation>();
    rclcpp::spin(radar_station);
    rclcpp::shutdown();
    return 0;
}


