#include "../include/robot_tracker/robot_tracker.hpp"

RobotTracker::RobotTracker() : Node("robot_tracker"){


    RCLCPP_INFO(this->get_logger(), "RobotTracker node has been created");
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto robot_tracker = std::make_shared<RobotTracker>();
    rclcpp::spin(robot_tracker);
    rclcpp::shutdown();
    return 0;
}