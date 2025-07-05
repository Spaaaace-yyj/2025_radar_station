//user
#include "../include/radar_station/radar_station.hpp"

using namespace std;

RadarStation::RadarStation() : Node("radar_station")
{

    init_parameters();

    generate_color_map();

    load_onnx_model();

    broadcaster_tf_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/lidar", 1, std::bind(&RadarStation::point_cloud_callback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&RadarStation::image_callback, this, std::placeholders::_1));
    
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/radar_station/point_cloud_debug", 10);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/radar_station/image_debug/point_cloud_projection", 10);

    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/radar_station/marker_array", 10);

    robot_position_array_pub_ = this->create_publisher<radar_station_interface::msg::RobotPositionArray>("/radar_station/robot_position_array", 10);

    RCLCPP_INFO(this->get_logger(), "Radar station node has been created");
}
void RadarStation::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    update_parameters();

    if (!point_cloud_ || point_cloud_->empty()) {
        RCLCPP_WARN(this->get_logger(), "Point cloud is empty or null");
        return;
    }
    if(lidar_frame_counter_ >= lidar_frame_add_num_){
        lidar_frame_counter_ = 0;
        
        cv::Mat image_mat_(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
        
        if(flip_image_){
            cv::rotate(image_mat_, frame_, cv::ROTATE_180);
            cv::rotate(image_mat_, onnx_debug_frame_, cv::ROTATE_180);
        }else{
            frame_ = image_mat_.clone();
            onnx_debug_frame_ = image_mat_.clone();
        }
        cv::cvtColor(frame_, frame_, cv::COLOR_BGR2RGB);
        cv::cvtColor(onnx_debug_frame_, onnx_debug_frame_, cv::COLOR_BGR2RGB);

        if(save_cloud_and_image_ == 1){
            std::string cloud_filename = file_path + "/" + std::to_string(save_count) + ".pcd";
            std::string image_filename = file_path + "/" + std::to_string(save_count) + ".jpg";
            if(save_image_only){
                cv::imwrite(image_filename, frame_);
            }else{
                pcl::io::savePCDFileBinary(cloud_filename, *point_cloud_);
                cv::imwrite(image_filename, frame_);
            }
            save_count++;
            std::cout << "Save point cloud and image" << save_count << std::endl;
            cv::waitKey(500);
        }


        start_image_time_ = cv::getTickCount();
        std::vector<OnnxBox> onnx_boxes = get_armor_box(frame_);
        
        end_image_time_ = cv::getTickCount();	

        Eigen::Matrix3f R_lidar_to_camera;

        // R_lidar_to_camera <<    -0.0318712393232544, -0.999474459707108, 0.00591848774478551,
        //                         0.0111232139656527, -0.00627581384510766, -0.999918440809877,
        //                         0.999430086706956, -0.0318028073252486, 0.0113173862335329;
        
        // Eigen::Vector3f T_lidar_to_camera(0.0428245893583569 + T_add_x_, 0.0104154355328730 + T_add_y_, -0.638969767020652 + T_add_z_);

        std::vector<cv::Point3f> lidar_points;
        std::vector<cv::Point2f> lidar_points_projection;
        
        for(size_t i = 0; i < point_cloud_->points.size(); i++){
            const auto& p = point_cloud_->points[i];

            //livox雷达坐标系与opencv坐标系定义不同，转换livox坐标系为opencv坐标系
            Eigen::Vector3f lidar_point(p.x, p.y, p.z);
            Eigen::Vector3f lidar_point_in_camera = R_lidar_to_camera_ * lidar_point + T_lidar_to_camera_;
            cv::Point3f point_lidar_in_camera(lidar_point_in_camera.x(), lidar_point_in_camera.y(), lidar_point_in_camera.z());

            lidar_points.push_back(point_lidar_in_camera);
        }

        cv::projectPoints(lidar_points, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, lidar_points_projection);
        
        
        //遍历屏幕投影点云，id与实际3D点云对应
        std::vector<float> min_dis_in_screen(onnx_boxes.size(), 10000);
        std::vector<int> min_dis_point_id(onnx_boxes.size());
        for(size_t i = 0; i < lidar_points_projection.size(); i++){
            //筛选点云roi
            for(size_t j = 0; j < onnx_boxes.size(); j++){
                if(lidar_points_projection[i].x > onnx_boxes[j].P1.x && lidar_points_projection[i].x < onnx_boxes[j].P2.x &&
                    lidar_points_projection[i].y > onnx_boxes[j].P1.y && lidar_points_projection[i].y < onnx_boxes[j].P2.y){
                    float dis = std::sqrt(std::pow(lidar_points_projection[i].x - onnx_boxes[j].center.x, 2) + std::pow(lidar_points_projection[i].y - onnx_boxes[j].center.y, 2));
                    if(dis < min_dis_in_screen[j]){
                        min_dis_in_screen[j] = dis;
                        min_dis_point_id[j] = i;
                    }
                }
            }
        }

        cloud_pub_->header = point_cloud_->header;
        cloud_pub_->is_dense = point_cloud_->is_dense;
        cloud_pub_->clear();
        for(size_t i = 0; i < lidar_points.size(); i++){
            pcl::PointXYZRGB colored_point;
            Eigen::Matrix3f R_camera_world = R_world_camera_.transpose();
            Eigen::Vector3f T_camera_world = - R_camera_world * T_world_camera_;
            Eigen::Vector3f world_pos;
            Eigen::Vector3f real_pos(lidar_points[i].x, lidar_points[i].y, lidar_points[i].z);
            world_pos = R_camera_world * real_pos + T_camera_world;
            colored_point.x = world_pos.x();
            colored_point.y = world_pos.y();
            colored_point.z = world_pos.z();
            colored_point.r = 0;
            colored_point.g = 0;
            colored_point.b = 255;
            for(size_t j = 0; j < min_dis_point_id.size(); j++){
                float dis_3D = std::sqrt(std::pow(lidar_points[i].x - lidar_points[min_dis_point_id[j]].x, 2) + std::pow(lidar_points[i].y - lidar_points[min_dis_point_id[j]].y, 2) + std::pow(lidar_points[i].z - lidar_points[min_dis_point_id[j]].z, 2));
                if(dis_3D < range_of_roi_){
                    target_[j].robot_points_roi_.push_back(lidar_points[i]);
                    target_[j].get_real_pos();
                    target_[j].get_world_location(R_world_camera_, T_world_camera_);
                    colored_point.r = 255;
                    colored_point.g = 0;
                    colored_point.b = 0;
                }
            }    
            cloud_pub_->push_back(colored_point);
        }

        std::vector<cv::Point2f> point_3d_roi_projection;
        for(size_t i = 0; i < target_.size(); i++){
            cv::projectPoints(target_[i].robot_points_roi_, cv::Mat::zeros(3, 1, CV_64F), cv::Mat::zeros(3, 1, CV_64F), cameraMatrix, distCoeffs, point_3d_roi_projection);
            for(size_t j = 0; j < point_3d_roi_projection.size(); j++){
                cv::circle(frame_, point_3d_roi_projection[j], 1.5, cv::Scalar(0, 0, 255), -1);
            }
        }

	    dt = (end_image_time_ - start_image_time_) * 1000 / cv::getTickFrequency();
        //publish_cloud_debug();  
        cv::putText(frame_, "Latency: " + to_string(dt) + "ms", cv::Point2f(5, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2, 8);
        
    }
    publish_image();
    publish_cloud();
    publish_marker_array();
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
    //RCLCPP_INFO(this->get_logger(), "frame_count = %d", lidar_frame_counter_);
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
    if(!frame_.empty()){
        std_msgs::msg::Header header;
        header.stamp = this->get_clock()->now();
        header.frame_id = "camera_frame";
        sensor_msgs::msg::Image::SharedPtr msg =
                cv_bridge::CvImage(header, "bgr8", frame_).toImageMsg();
        image_pub_->publish(*msg);
    }
}

void RadarStation::publish_cloud(){
    cloud_pub_->width = cloud_pub_->points.size();
    cloud_pub_->height = 1;
    sensor_msgs::msg::PointCloud2 output_msg;
    pcl::toROSMsg(*cloud_pub_, output_msg);
    output_msg.header.stamp = this->get_clock()->now();
    output_msg.header.frame_id = "map";
    point_cloud_pub_->publish(output_msg);
}

std::vector<OnnxBox> RadarStation::get_armor_box(const cv::Mat& src){
    std::vector<OnnxBox> onnx_boxes_car;
    std::vector<OnnxBox> onnx_boxes_armor;
    std::vector<Robot> target_temp;
    if(!src.empty()){
        cv::Mat src_resize;
        cv::resize(src, src_resize, cv::Size(640, 640));
        
        cv::Mat blob_car = cv::dnn::blobFromImage(src_resize, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);
        car_net.setInput(blob_car);
        std::vector<cv::Mat> outs_car;
        std::vector<std::string> outNames = { "output0" };
        car_net.forward(outs_car, outNames);
        
        onnx_boxes_car = process_onnx_result(outs_car[0], src);

        for(size_t i = 0; i < onnx_boxes_car.size(); i++){
            Robot robot_temp;
            cv::Rect roi(onnx_boxes_car[i].P1, onnx_boxes_car[i].P2);
            cv::Mat roi_img = src(roi);
            cv::Mat roi_img_resize;
            cv::resize(roi_img, roi_img_resize, cv::Size(640, 640));

            cv::Mat blob_armor = cv::dnn::blobFromImage(roi_img_resize, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);
            armor_net.setInput(blob_armor);
            std::vector<cv::Mat> outs_armor;
            std::vector<std::string> outNames = { "output0" };
            armor_net.forward(outs_armor, outNames);

            onnx_boxes_armor = process_onnx_result(outs_armor[0], roi_img);
            //修复如果没有装甲板识别到的情况
            if(!onnx_boxes_armor.empty()){
                onnx_boxes_car[i].class_id = onnx_boxes_armor[0].class_id;
                robot_temp.id_ = onnx_boxes_car[i].class_id;
                target_temp.push_back(robot_temp);
            }else{
                onnx_boxes_car[i].class_id = -1;
                robot_temp.id_ = -1;
                target_temp.push_back(robot_temp);
            }
            
        }
        target_ = target_temp;
        for(size_t i = 0; i < onnx_boxes_car.size(); i++){
            onnx_boxes_car[i].draw(frame_);
        }

        return onnx_boxes_car;
    }else{
        RCLCPP_WARN(this->get_logger(), "Can't get armor box. Because the image is empty.");
        return std::vector<OnnxBox>();
    }
}

std::vector<OnnxBox> RadarStation::process_onnx_result(const cv::Mat& onnx_result, cv::Mat src){
    int num_attributes = onnx_result.size[1];
    int num_anchors = onnx_result.size[2];
    std::vector<OnnxBox> onnx_boxes_temp;

    float* data = (float*)onnx_result.data;
        for(int j = 0; j < num_attributes; j++){
            if(data[j * num_anchors + 4] > 0.6){
                OnnxBox box;

                float x_center = (data[j * num_anchors + 0] / 640) * src.cols;
                float y_center = (data[j * num_anchors + 1] / 640) * src.rows;
                float w = (data[j * num_anchors + 2] / 640) * src.cols;
                float h = (data[j * num_anchors + 3] / 640) * src.rows;

                box.P1 = cv::Point2f(x_center - w / 2, y_center - h / 2);
                box.P2 = cv::Point2f(x_center + w / 2, y_center + h / 2);
                box.width = w;
                box.height = h;
                box.center = cv::Point2f(x_center, y_center);

                float max_class_conf = 0;
                int class_id_temp = 1;
                for(int i = 1; i < 5; i++){
                    if(data[j * num_anchors + 4 + i] * data[j * num_anchors + 4] > max_class_conf){
                        max_class_conf = data[j * num_anchors + 4 + i] * data[j * num_anchors + 4];
                        box.class_id = i;
                        class_id_temp = i;
                    }
                }
                box.conf = max_class_conf;
                box.class_id = class_id_temp;
                onnx_boxes_temp.push_back(box);
            }else{
                continue;
            }
        }
        nms(onnx_boxes_temp, 0.5);

        return onnx_boxes_temp;
}

float RadarStation::get_iou(OnnxBox box1, OnnxBox box2){
    float x1 = std::max(box1.P1.x, box2.P1.x);
    float y1 = std::max(box1.P1.y, box2.P1.y);
    float x2 = std::min(box1.P2.x, box2.P2.x);
    float y2 = std::min(box1.P2.y, box2.P2.y);

    float area1 = box1.width * box1.height;
    float area2 = box2.width * box2.height;

    float inter_area = std::max(0.0f, x2 - x1) * std::max(0.0f, y2 - y1);

    float iou = inter_area / (area1 + area2 - inter_area);

    return iou;
}

void RadarStation::nms(std::vector<OnnxBox>& boxes, float iou_threshold){
    std::sort(boxes.begin(), boxes.end(), [](OnnxBox box1, OnnxBox box2){
        return box1.conf > box2.conf;
    });

    std::vector<OnnxBox> boxes_temp;
    for(size_t i = 0; i < boxes.size(); i++){
        bool is_suppressed = false;
        for(size_t j = 0; j < boxes_temp.size(); j++){
            if(get_iou(boxes[i], boxes_temp[j]) > iou_threshold){
                is_suppressed = true;
                break;
            }
        }
        if(!is_suppressed){
            boxes_temp.push_back(boxes[i]);
        }
    }
    boxes = boxes_temp;
}

void RadarStation::load_onnx_model(){
    RCLCPP_INFO(this->get_logger(), "Loading onnx model...");
    car_net = cv::dnn::readNet(car_model_path_);
    armor_net = cv::dnn::readNet(armor_model_path_);

    car_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    car_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    armor_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    armor_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
}

void RadarStation::init_parameters(){
    RCLCPP_INFO(this->get_logger(), "Initializing parameters...");
    this->declare_parameter("save_cloud_and_image", 0);
    this->declare_parameter("range_of_roi", 0.35f);
    this->declare_parameter("camera.camera_matrix.data", std::vector<double>(9, 0.0));
    this->declare_parameter("camera.distortion_coefficients.data", std::vector<double>(5, 0.0));
    this->declare_parameter("T_radar2camera.T_add_x", -0.02f);
    this->declare_parameter("T_radar2camera.T_add_y", 0.02f);
    this->declare_parameter("T_radar2camera.T_add_z", 0.6f);
    this->declare_parameter("T_radar2camera.Rotation.data", std::vector<double>(9, 0.0));
    this->declare_parameter("T_radar2camera.Translation.data", std::vector<double>(3, 0.0));
    this->declare_parameter("T_world2camera.Rotation.data", std::vector<double>(9, 0.0));
    this->declare_parameter("T_world2camera.Translation.data", std::vector<double>(3, 0.0));
    update_parameters();
}


void RadarStation::update_parameters(){
    this->get_parameter("T_radar2camera.T_add_x", T_add_x_);
    this->get_parameter("T_radar2camera.T_add_y", T_add_y_);
    this->get_parameter("T_radar2camera.T_add_z", T_add_z_);
    this->get_parameter("range_of_roi", range_of_roi_);


    std::vector<double> R_lidar2camera;
    std::vector<double> T_lidar2camera;
    this->get_parameter("T_radar2camera.Rotation.data", R_lidar2camera);
    this->get_parameter("T_radar2camera.Translation.data", T_lidar2camera);
    R_lidar_to_camera_ << R_lidar2camera[0], R_lidar2camera[1], R_lidar2camera[2],
        R_lidar2camera[3], R_lidar2camera[4], R_lidar2camera[5],
        R_lidar2camera[6], R_lidar2camera[7], R_lidar2camera[8];
    T_lidar_to_camera_ << T_lidar2camera[0], T_lidar2camera[1], T_lidar2camera[2];
    T_lidar_to_camera_.x() += T_add_x_;
    T_lidar_to_camera_.y() += T_add_y_;
    T_lidar_to_camera_.z() += T_add_z_;

    std::vector<double> camera_matrix;
    std::vector<double> distortion_coefficients;
    this->get_parameter("camera.camera_matrix.data", camera_matrix);
    this->get_parameter("camera.distortion_coefficients.data", distortion_coefficients);
    cameraMatrix = (cv::Mat_<double>(3, 3) << camera_matrix[0], camera_matrix[1], camera_matrix[2],
        camera_matrix[3], camera_matrix[4], camera_matrix[5],
        camera_matrix[6], camera_matrix[7], camera_matrix[8]);
    distCoeffs = (cv::Mat_<double>(5, 1) << distortion_coefficients[0], distortion_coefficients[1], distortion_coefficients[2],
        distortion_coefficients[3], distortion_coefficients[4]);

    std::vector<double> R_world2camera;
    std::vector<double> T_world2camera;
    this->get_parameter("T_world2camera.Rotation.data", R_world2camera);
    this->get_parameter("T_world2camera.Translation.data", T_world2camera);
    R_world_camera_ << R_world2camera[0], R_world2camera[1], R_world2camera[2],
        R_world2camera[3], R_world2camera[4], R_world2camera[5],
        R_world2camera[6], R_world2camera[7], R_world2camera[8];
    T_world_camera_ << T_world2camera[0], T_world2camera[1], T_world2camera[2];
}   

void RadarStation::publish_marker_array(){
    visualization_msgs::msg::MarkerArray marker_array;

    for(size_t i = 0; i < target_.size(); i++){
        visualization_msgs::msg::Marker marker_box;
        marker_box.header.frame_id = "map";
        marker_box.header.stamp = this->get_clock()->now();
        marker_box.ns = "target_box" + std::to_string(i);
        marker_box.id = i; 
        marker_box.type = visualization_msgs::msg::Marker::CUBE;
        marker_box.action = visualization_msgs::msg::Marker::ADD;

        marker_box.pose.position.x = target_[i].world_pos_.x;
        marker_box.pose.position.y = target_[i].world_pos_.y;
        marker_box.pose.position.z = target_[i].world_pos_.z;
        marker_box.pose.orientation.w = 1.0;

        marker_box.scale.x = target_[i].width_;
        marker_box.scale.z = target_[i].height_;
        marker_box.scale.y = target_[i].depth_;

        marker_box.color.r = 0.1f * i;
        marker_box.color.g = 0.5f;
        marker_box.color.b = 1.0f - 0.2f * i;
        marker_box.color.a = 0.5f;
        marker_box.lifetime = rclcpp::Duration::from_seconds(1); // 永久显示
        marker_array.markers.push_back(marker_box);

        visualization_msgs::msg::Marker marker_center;
        marker_center.header.frame_id = "map";
        marker_center.header.stamp = this->get_clock()->now();
        marker_center.ns = "target_center" + std::to_string(i);
        marker_center.id = i;
        marker_center.type = visualization_msgs::msg::Marker::SPHERE;
        marker_center.action = visualization_msgs::msg::Marker::ADD;

        marker_center.pose.position.x = target_[i].world_pos_.x;
        marker_center.pose.position.y = target_[i].world_pos_.y;
        marker_center.pose.position.z = target_[i].world_pos_.z;
        marker_center.pose.orientation.w = 1.0;

        marker_center.scale.x = 0.1;
        marker_center.scale.y = 0.1;
        marker_center.scale.z = 0.1;

        marker_center.color.r = 1.0f;
        marker_center.color.g = 1.0f;
        marker_center.color.b = 0.0f;
        marker_center.color.a = 1.0f; 
        marker_center.lifetime = rclcpp::Duration::from_seconds(1); 
        marker_array.markers.push_back(marker_center);
        
        visualization_msgs::msg::Marker marker_text;
        marker_text.header.frame_id = "map";
        marker_text.header.stamp = this->get_clock()->now();
        marker_text.ns = "target_text" + std::to_string(i);
        marker_text.id = i;
        marker_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        marker_text.action = visualization_msgs::msg::Marker::ADD;

        marker_text.pose.position.x = target_[i].world_pos_.x;
        marker_text.pose.position.y = target_[i].world_pos_.y;
        marker_text.pose.position.z = target_[i].world_pos_.z + 0.5;

        marker_text.scale.z = 0.1;

        marker_text.color.r = 1.0f;
        marker_text.color.g = 1.0f;
        marker_text.color.b = 0.0f;
        marker_text.color.a = 1.0f;  // 不透明
        marker_text.text = "ID:" + std::to_string(target_[i].id_) + "\n" + 
                            " x:" + std::to_string(target_[i].world_pos_.x) + "\n" + 
                            " y:" + std::to_string(target_[i].world_pos_.y) + "\n" + 
                            " z:" + std::to_string(target_[i].world_pos_.z);
        marker_text.lifetime = rclcpp::Duration::from_seconds(1); // 永久显示
        marker_array.markers.push_back(marker_text);
    }
    cv::Mat R_world_camera_cv = (cv::Mat_<double>(3, 3) << R_world_camera_(0, 0), R_world_camera_(0, 1), R_world_camera_(0, 2),
                                                            R_world_camera_(1, 0), R_world_camera_(1, 1), R_world_camera_(1, 2),
                                                            R_world_camera_(2, 0), R_world_camera_(2, 1), R_world_camera_(2, 2));
    cv::Mat T_world_camera_cv = (cv::Mat_<double>(3, 1) << T_world_camera_(0, 0), T_world_camera_(1, 0), T_world_camera_(2, 0));

    // 发布变换
    publish_transform(R_world_camera_cv, T_world_camera_cv, "map", "camera_link");

    marker_array_pub_->publish(marker_array);
}

void RadarStation::publish_transform(
    	const cv::Mat& rvec,     //rotation
    	const cv::Mat& tvec,     //transmision
    	const std::string& frame_id, //world frame
    	const std::string& child_frame_id ) {
			// publish translation and rotation information
			cv::Mat rvecs = rvec;  // 旋转向量（3x1矩阵）
			cv::Mat R;
			cv::Rodrigues(rvecs, R);  // 转换为3x3旋转矩阵

			// 将OpenCV矩阵转换为Eigen矩阵
			Eigen::Matrix3d eigen_R;
			eigen_R << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
            R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
	        R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);

			// 从旋转矩阵创建四元数
			Eigen::Quaterniond q(eigen_R);

    		// 创建静态坐标变换（世界坐标系 -> 相机坐标系）
    		geometry_msgs::msg::TransformStamped transform_stamped;
    		transform_stamped.header.stamp = this->get_clock()->now();
    		transform_stamped.header.frame_id = frame_id; // 世界坐标系名称（需与 RViz 一致）
    		transform_stamped.child_frame_id = child_frame_id; // 相机坐标系名称

    		// 设置平移
    		transform_stamped.transform.translation.x = tvec.at<double>(0, 0);
    		transform_stamped.transform.translation.y = -tvec.at<double>(2, 0);
    		transform_stamped.transform.translation.z = tvec.at<double>(1, 0);

    		// 设置旋转（四元数）
    		transform_stamped.transform.rotation.x = q.x();
    		transform_stamped.transform.rotation.y = q.y();
    		transform_stamped.transform.rotation.z = q.z();
    		transform_stamped.transform.rotation.w = q.w();

    				
    		broadcaster_tf_->sendTransform(transform_stamped);
	}

void RadarStation::publish_robot_position_array(){
    for(size_t i = 0; i < target_.size(); i++){
        radar_station_interface::msg::RobotPosition robot_info;
        robot_info.x = target_[i].world_pos_.x;
        robot_info.y = target_[i].world_pos_.y;
        robot_info.z = target_[i].world_pos_.z;
        robot_info.id = target_[i].id_;
    }
    
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto radar_station = std::make_shared<RadarStation>();
    rclcpp::spin(radar_station);
    rclcpp::shutdown();
    return 0;
}


