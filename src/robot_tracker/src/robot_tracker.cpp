#include "../include/robot_tracker/robot_tracker.hpp"

RobotTracker::RobotTracker() : Node("robot_tracker"){

    init_params();

    robot_position_array_sub_ = this->create_subscription<radar_station_interface::msg::RobotPositionArray>(
        "radar_station/robot_position_array", 10, std::bind(&RobotTracker::robot_position_callback, this, std::placeholders::_1));
    
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/robot_tracker/marker_array", 10);

    RCLCPP_INFO(this->get_logger(), "RobotTracker node has been created");
}

void RobotTracker::robot_position_callback(const radar_station_interface::msg::RobotPositionArray::SharedPtr msg){
    update_params();
    for(size_t i = 0; i < msg->positions.size(); i++){
        
    }
    publish_marker_array();
}

void RobotTracker::init_params(){
    this->declare_parameter("alpha_q", 0.01);
    this->declare_parameter("alpha_r", 0.00001);
    update_params();
}

void RobotTracker::update_params(){
    alpha_q_ = this->get_parameter("alpha_q").as_double();
    alpha_r_ = this->get_parameter("alpha_r").as_double();
}

void RobotTracker::publish_marker_array(){

}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto robot_tracker = std::make_shared<RobotTracker>();
    rclcpp::spin(robot_tracker);
    rclcpp::shutdown();
    return 0;
}