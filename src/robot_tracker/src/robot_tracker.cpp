#include "../include/robot_tracker/robot_tracker.hpp"

RobotTracker::RobotTracker() : Node("robot_tracker"){

    init_params();

    robot_position_array_sub_ = this->create_subscription<radar_station_interface::msg::RobotPositionArray>(
        "radar_station/robot_position_array", 10, std::bind(&RobotTracker::robot_position_callback, this, std::placeholders::_1));
    
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/robot_tracker/marker_array", 10);

    timer_ = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&RobotTracker::publish_marker_array, this)
        );

    init_robot_list();
    RCLCPP_INFO(this->get_logger(), "RobotTracker node has been created");
}

void RobotTracker::robot_position_callback(const radar_station_interface::msg::RobotPositionArray::SharedPtr msg){
    update_params();
    for(size_t i = 0; i < robot_list_.size(); i++){
        Robot::InputState input_state = Robot::InputState::LOST_INPUT;
        for(size_t j = 0; j < msg->positions.size(); j++){
            if(Robot::RobotID(msg->positions[j].id) == robot_list_[i].id_){
                input_state = Robot::InputState::TRACKING_INPUT;
                robot_list_[i].measurement_position_.x = msg->positions[j].x;
                robot_list_[i].measurement_position_.y = msg->positions[j].y;
                robot_list_[i].measurement_position_.z = msg->positions[j].z;
                robot_list_[i].width_ = msg->positions[j].width;
                robot_list_[i].height_ = msg->positions[j].height;
                robot_list_[i].depth_ = msg->positions[j].depth;
                robot_list_[i].update_state(dt_, input_state);
                break;
            }
        }
        if(input_state == Robot::InputState::LOST_INPUT){
            robot_list_[i].measurement_position_.x = 0;
            robot_list_[i].measurement_position_.y = 0;
            robot_list_[i].measurement_position_.z = 0;
            robot_list_[i].update_state(dt_, Robot::InputState::LOST_INPUT);
        }
    }
    //todo list : 1.尝试采用匈牙利算法匹配机器人
    //            2.用累积投票来确定机器人ID，解决跳变问题

}

float RobotTracker::get_iou(const Robot& r1, const Robot& r2){
    float x1 = std::max(r1.predict_position_.x - r1.width_ / 2, r2.predict_position_.x - r2.width_ / 2);
    float x2 = std::min(r1.predict_position_.x + r1.width_ / 2, r2.predict_position_.x + r2.width_ / 2);
    float y1 = std::max(r1.predict_position_.y - r1.height_ / 2, r2.predict_position_.y - r2.height_ / 2);
    float y2 = std::min(r1.predict_position_.y + r1.height_ / 2, r2.predict_position_.y + r2.height_ / 2);
    float z1 = std::max(r1.predict_position_.z - r1.depth_ / 2, r2.predict_position_.z - r2.depth_ / 2);
    float z2 = std::min(r1.predict_position_.z + r1.depth_ / 2, r2.predict_position_.z + r2.depth_ / 2);

    float volume_public = (x2 - x1) * (y2 - y1) * (z2 - z1);
    float volume_r1 = (r1.width_ * r1.height_ * r1.depth_);
    float volume_r2 = (r2.width_ * r2.height_ * r2.depth_);
    if(volume_public <= 0 || volume_r1 <= 0 || volume_r2 <= 0){
        return 0.0f;
    }
    float iou = volume_public / (volume_r1 + volume_r2 - volume_public);
    return iou;
}

std::vector<std::vector<float>> RobotTracker::get_cost_matrix(const std::vector<Robot>& robot_list1, std::vector<Robot>& robot_list2){
    std::vector<std::vector<float>> cost_matrix(robot_list1.size(), std::vector<float>(robot_list2.size(), 0));
    for(size_t i = 0; i < robot_list1.size(); i++){
        for(size_t j = 0; j < robot_list2.size(); j++){
            cost_matrix[i][j] = 1 - get_iou(robot_list1[i], robot_list2[j]);
        }
    }
    return cost_matrix;
}

void RobotTracker::init_params(){

    update_params();
}

void RobotTracker::update_params(){

}

void RobotTracker::publish_marker_array(){
    visualization_msgs::msg::MarkerArray marker_array;
    for(size_t i = 0; i < robot_list_.size(); i++){
        if(robot_list_[i].state_ == Robot::LOST){
            continue;
        }

        visualization_msgs::msg::Marker marker_box;
        marker_box.header.frame_id = "map";
        marker_box.header.stamp = this->get_clock()->now();
        marker_box.ns = "target_box_predict_" + std::to_string(i);
        marker_box.id = i; 
        marker_box.type = visualization_msgs::msg::Marker::CUBE;
        marker_box.action = visualization_msgs::msg::Marker::ADD;

        marker_box.pose.position.x = robot_list_[i].predict_position_.x;
        marker_box.pose.position.y = robot_list_[i].predict_position_.y;
        marker_box.pose.position.z = robot_list_[i].predict_position_.z;
        marker_box.pose.orientation.w = 1.0;

        marker_box.scale.x = robot_list_[i].height_;
        marker_box.scale.z = robot_list_[i].width_;
        marker_box.scale.y = robot_list_[i].depth_;

        if(robot_list_[i].state_ == Robot::TRACKING){
            marker_box.color.r = 0.1f;
            marker_box.color.g = 0.5f;
            marker_box.color.b = 0.8f;
            marker_box.color.a = 0.5f;
        }else if(robot_list_[i].state_ == Robot::PREDICTING){
            marker_box.color.r = 0.1f;
            marker_box.color.g = 0.5f;
            marker_box.color.b = 0.1f;
            marker_box.color.a = 0.5f;
        }
        marker_box.lifetime = rclcpp::Duration::from_seconds(1); 
        marker_array.markers.push_back(marker_box);

        visualization_msgs::msg::Marker marker_center;
        marker_center.header.frame_id = "map";
        marker_center.header.stamp = this->get_clock()->now();
        marker_center.ns = "target_center_predict_" + std::to_string(i);
        marker_center.id = i;
        marker_center.type = visualization_msgs::msg::Marker::SPHERE;
        marker_center.action = visualization_msgs::msg::Marker::ADD;
        marker_center.pose.position.x = robot_list_[i].predict_position_.x;
        marker_center.pose.position.y = robot_list_[i].predict_position_.y;
        marker_center.pose.position.z = robot_list_[i].predict_position_.z;
        marker_center.pose.orientation.w = 1.0;
        marker_center.scale.x = 0.1;
        marker_center.scale.y = 0.1;
        marker_center.scale.z = 0.1;
        marker_center.color.r = 1.0f;
        marker_center.color.g = 0.0f;
        marker_center.color.b = 0.0f;
        marker_center.color.a = 0.5f; 
        marker_center.lifetime = rclcpp::Duration::from_seconds(3); 
        marker_array.markers.push_back(marker_center);

        visualization_msgs::msg::Marker marker_text;
        marker_text.header.frame_id = "map";
        marker_text.header.stamp = this->get_clock()->now();
        marker_text.ns = "target_text_predict_" + std::to_string(i);
        marker_text.id = i;
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;

        marker_text.pose.position.x = robot_list_[i].predict_position_.x;
        marker_text.pose.position.y = robot_list_[i].predict_position_.y;
        marker_text.pose.position.z = robot_list_[i].predict_position_.z + 0.5;

        marker_text.scale.z = 0.1;

        marker_text.color.r = 1.0f;
        marker_text.color.g = 1.0f;
        marker_text.color.b = 0.0f;
        marker_text.color.a = 1.0f; 
        if(robot_list_[i].state_ == Robot::TRACKING){
            marker_text.text = "ID:" + std::to_string(robot_list_[i].id_) + "\n" + 
                    " x:" + std::to_string(robot_list_[i].predict_position_.x) + "\n" + 
                    " y:" + std::to_string(robot_list_[i].predict_position_.y) + "\n" + 
                    " z:" + std::to_string(robot_list_[i].predict_position_.z) + "\n" + 
                    "Tracking----";
        }else{
            marker_text.text = "ID:" + std::to_string(robot_list_[i].id_) + "\n" + 
                    " x:" + std::to_string(robot_list_[i].predict_position_.x) + "\n" + 
                    " y:" + std::to_string(robot_list_[i].predict_position_.y) + "\n" + 
                    " z:" + std::to_string(robot_list_[i].predict_position_.z) + "\n" +
                    "Predicting----";
        }

        marker_text.lifetime = rclcpp::Duration::from_seconds(1); 
        marker_array.markers.push_back(marker_text);

    }
    marker_array_pub_->publish(marker_array);
}

void RobotTracker::init_robot_list(){
    robot_list_.clear();
    for(int i = 0; i < 10; i++){
        Robot robot;
        robot.init(dt_, Robot::RobotID(i));
        robot_list_.push_back(robot);
    }
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto robot_tracker = std::make_shared<RobotTracker>();
    rclcpp::spin(robot_tracker);
    rclcpp::shutdown();
    return 0;
}