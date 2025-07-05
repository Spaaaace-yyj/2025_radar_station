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

class RobotTracker : public rclcpp::Node{
public:
    RobotTracker();
    ~RobotTracker() = default;

private:



};


#endif // ROBOT_TRACKER_HPP_
