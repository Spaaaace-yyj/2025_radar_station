//opencv
#include <opencv4/opencv2/opencv.hpp>
//ros2
#include <rclcpp/rclcpp.hpp>
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/filters/voxel_grid.h"
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
        //declare _param
        this->declare_parameter("image_width");
        this->declare_parameter("image_height");
        this->declare_parameter<std::vector<double>>("camera_matrix.data");
        this->get_parameter("image_width", image_width_);
        this->get_parameter("image_height", image_height_);
        this->get_parameter("camera_matrix.data", camera_matrix_);


        point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "livox/lidar", 10, std::bind(&RadarStation::point_cloud_callback, this, std::placeholders::_1));
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera_info", 10, std::bind(&RadarStation::camera_info_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "Radar station node has been created");
    }

private:
    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Radar station node has received a point cloud");
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*msg, *cloud);

        for(size_t i = 0; i < std::min(cloud->points.size(), size_t(10)); i++){
            const auto& pt = cloud->points[i];
            std::cout << "x: " << pt.x << " y: " << pt.y << " z: " << pt.z << " intensity: " << pt.intensity << std::endl;
        }

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

private:
    int image_width_;
    int image_height_;
    std::vector<double> camera_matrix_;

};



int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto radar_station = std::make_shared<RadarStation>();
    rclcpp::spin(radar_station);
    rclcpp::shutdown();
    return 0;
}


