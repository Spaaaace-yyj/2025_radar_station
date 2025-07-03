//user
#include "../include/radar_station/radar_station.hpp"

using namespace std;

RadarStation::RadarStation() : Node("radar_station")
{
    this->declare_parameter("save_cloud_and_image", 0);
    this->declare_parameter("T_add_x", -0.02f);
    this->declare_parameter("T_add_y", 0.02f);
    this->declare_parameter("T_add_z", 0.6f);
    this->declare_parameter("range_of_roi", 0.35f);

    this->get_parameter("save_cloud_and_image", save_cloud_and_image_);

    generate_color_map();

    load_onnx_model();

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "livox/lidar", 1, std::bind(&RadarStation::point_cloud_callback, this, std::placeholders::_1));
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("/image_raw", 10, std::bind(&RadarStation::image_callback, this, std::placeholders::_1));
    
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/radar_station/point_cloud_debug", 10);

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/radar_station/image_debug/point_cloud_projection", 10);

    onnx_debug_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/radar_station/image_debug/onnx_result", 10);


    RCLCPP_INFO(this->get_logger(), "Radar station node has been created");
}
void RadarStation::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    this->get_parameter("save_cloud_and_image", save_cloud_and_image_);
    this->get_parameter("T_add_x", T_add_x_);
    this->get_parameter("T_add_y", T_add_y_);
    this->get_parameter("T_add_z", T_add_z_);
    this->get_parameter("range_of_roi", range_of_roi_);

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

        openvino_test(onnx_debug_frame_);

        Eigen::Matrix3f R_lidar_to_camera;

        R_lidar_to_camera <<    -0.0318712393232544, -0.999474459707108, 0.00591848774478551,
                                0.0111232139656527, -0.00627581384510766, -0.999918440809877,
                                0.999430086706956, -0.0318028073252486, 0.0113173862335329;
        
        Eigen::Vector3f T_lidar_to_camera(0.0428245893583569 + T_add_x_, 0.0104154355328730 + T_add_y_, -0.638969767020652 + T_add_z_);

        std::vector<cv::Point3f> lidar_points;
        std::vector<cv::Point2f> lidar_points_projection;
        
        for(size_t i = 0; i < point_cloud_->points.size(); i++){
            const auto& p = point_cloud_->points[i];

            //livox雷达坐标系与opencv坐标系定义不同，转换livox坐标系为opencv坐标系
            Eigen::Vector3f lidar_point(p.x, p.y, p.z);
            Eigen::Vector3f lidar_point_in_camera = R_lidar_to_camera * lidar_point + T_lidar_to_camera;
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
            colored_point.x = lidar_points[i].x;
            colored_point.y = lidar_points[i].y;
            colored_point.z = lidar_points[i].z;
            colored_point.r = 0;
            colored_point.g = 0;
            colored_point.b = 255;
            for(size_t j = 0; j < min_dis_point_id.size(); j++){
                float dis_3D = std::sqrt(std::pow(lidar_points[i].x - lidar_points[min_dis_point_id[j]].x, 2) + std::pow(lidar_points[i].y - lidar_points[min_dis_point_id[j]].y, 2) + std::pow(lidar_points[i].z - lidar_points[min_dis_point_id[j]].z, 2));
                if(dis_3D < range_of_roi_){
                    target_[j].robot_points_roi_.push_back(lidar_points[i]);
                    target_[j].get_real_pos();
                    float dis = std::sqrt(std::pow(target_[j].real_pos_.x, 2) + std::pow(target_[j].real_pos_.y, 2) + std::pow(target_[j].real_pos_.z, 2));
                    //std::cout << "dis = " << dis << std::endl;
                    colored_point.r = 255;
                    colored_point.g = 0;
                    colored_point.b = 0;
                }else{
                    colored_point.r = 0;
                    colored_point.g = 255;
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


void RadarStation::publish_onnx_debug_image(){
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "livox_frame";
    sensor_msgs::msg::Image::SharedPtr msg =
            cv_bridge::CvImage(header, "bgr8", onnx_debug_frame_).toImageMsg();
    onnx_debug_pub_->publish(*msg);
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
            if(data[j * num_anchors + 4] > 0.7){
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
                publish_onnx_debug_image();
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

ov::Tensor RadarStation::convert_mat_to_f16_tensor(const cv::Mat& float_rgb, const ov::Shape& shape) {
    ov::Tensor tensor(ov::element::f16, shape);
    ov::float16* data_ptr = tensor.data<ov::float16>();
    const float* src_ptr = reinterpret_cast<const float*>(float_rgb.data);

    size_t num_elements = shape[1] * shape[2] * shape[3];  // [N,C,H,W]
    for (size_t i = 0; i < num_elements; ++i) {
        data_ptr[i] = ov::float16(src_ptr[i]);  // 显式 float32 → float16
    }
    return tensor;
}

void RadarStation::openvino_test(cv::Mat src){
    ov::Output<const ov::Node> input_port = compiled_car_model_.input();
    auto input_shape = input_port.get_shape();  // e.g. [1, 3, 640, 640]

    cv::Mat resized;
    cv::resize(src, resized, cv::Size(input_shape[3], input_shape[2]));

    resized.convertTo(resized, CV_32F, 1.0 / 255.0);

    ov::Tensor input_tensor = convert_mat_to_f16_tensor(resized, input_shape);
    infer_request_car_.set_input_tensor(input_tensor);
    infer_request_car_.infer();

    ov::Tensor output = infer_request_car_.get_output_tensor();

    std::vector<float> output_data;

    if (output.get_element_type() == ov::element::f16) {
        const ov::float16* f16_data = output.data<const ov::float16>();
        size_t num_elements = output.get_size();
        output_data.resize(num_elements);
        for (size_t i = 0; i < num_elements; ++i) {
            output_data[i] = static_cast<float>(f16_data[i]);  // f16 -> f32
        }
    } else if (output.get_element_type() == ov::element::f32) {
        const float* data_ptr = output.data<const float>();
        output_data.assign(data_ptr, data_ptr + output.get_size());
    } else {
        throw std::runtime_error("Unsupported output tensor type!");
    }

    const auto& out_shape = output.get_shape(); // [1, N, M]
    size_t num_detections = out_shape[1];
    size_t dim = out_shape[2]; // YOLOv8 是 [1, 25200, 6~17]
    size_t num_classes = dim - 5;
    // for (size_t i = 0; i < num_detections; ++i) {
    //     float x = output_data[i * dim + 0];
    //     float y = output_data[i * dim + 1];
    //     float w = output_data[i * dim + 2];
    //     float h = output_data[i * dim + 3];
    //     float obj_conf = output_data[i * dim + 4];
    //     if(obj_conf < 1e-4) continue;
    //     cout << "x: " << x << " y: " << y << " w: " << w << " h: " << h << " obj_conf: " << obj_conf << endl;
    //     // 找出 class 分数最大的类别和置信度
    //     float max_cls_score = 0.0f;
    //     int class_id = -1;
    //     for (int j = 0; j < num_classes; ++j) {
    //         float cls_score = output_data[i * dim + 5 + j];
    //         if (cls_score > max_cls_score) {
    //             max_cls_score = cls_score;
    //             class_id = j;
    //         }
    //     }

    //     float conf = obj_conf * max_cls_score;
    //     if (conf < 1e-3) continue;
    //     // 坐标是相对的，乘图像尺寸
    //     int left = static_cast<int>((x - w / 2.0) * src.cols);
    //     int top = static_cast<int>((y - h / 2.0) * src.rows);
    //     int width = static_cast<int>(w * src.cols);
    //     int height = static_cast<int>(h * src.rows);

    //     cv::rectangle(src, cv::Rect(left, top, width, height), cv::Scalar(0, 255, 0), 2);
    //     std::string label = cv::format("cls:%d %.2f", class_id, conf);
    //     cv::putText(src, label, cv::Point(left, top - 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 0}, 1);
    //     }
    for(int i = 0; i < output_data.size(); i++){
        if(output_data[i] >= 0.1 && output_data[i] <= 1){
            cout << "output_data[" << i << "] = " << output_data[i] << endl;
        }
    }
        cv::imshow("openvino", src);
        cv::waitKey(1);
}

void RadarStation::load_onnx_model(){
    car_net = cv::dnn::readNet(car_model_path_);
    armor_net = cv::dnn::readNet(armor_model_path_);
    
    car_model_ = car_core_.read_model(car_model_path_openvino_);
    armor_model_ = armor_core_.read_model(armor_model_path_openvino_);
    
    compiled_car_model_ = car_core_.compile_model(car_model_, "AUTO");
    compiled_armor_model_ = armor_core_.compile_model(armor_model_, "AUTO");

    infer_request_car_ = compiled_car_model_.create_infer_request();
    infer_request_armor_ = compiled_armor_model_.create_infer_request();
    
}

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    auto radar_station = std::make_shared<RadarStation>();
    rclcpp::spin(radar_station);
    rclcpp::shutdown();
    return 0;
}


