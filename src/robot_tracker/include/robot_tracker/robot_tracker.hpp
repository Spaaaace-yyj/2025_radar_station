#ifndef ROBOT_TRACKER_HPP_
#define ROBOT_TRACKER_HPP_

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

#include "radar_station_interface/msg/robot_position.hpp"
#include "radar_station_interface/msg/robot_position_array.hpp"
#include "kalman_fliter.hpp"

class RobotTracker : public rclcpp::Node{
public:
    RobotTracker();
    ~RobotTracker() = default;

    void robot_position_callback(const radar_station_interface::msg::RobotPositionArray::SharedPtr msg);

    void publish_marker_array();

    void init_params();

    void update_params();

private:
    double alpha_q_ = 1.0;
    double alpha_r_ = 1.0;
    double dt_ = 0.1;

    Eigen::Vector4d pridict_position_;

    //temp test
    cv::Point3f robot_position_;
    cv::Point3f robot_velocity_;

    double width_;
    double height_ ;
    double depth_;

private:
    rclcpp::Subscription<radar_station_interface::msg::RobotPositionArray>::SharedPtr robot_position_array_sub_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;

};


#endif // ROBOT_TRACKER_HPP_
