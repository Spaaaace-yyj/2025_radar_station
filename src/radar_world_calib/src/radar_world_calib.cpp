#include "../include/radar_world_calib/radar_world_calib.hpp"

RadarWorldCalib::RadarWorldCalib() : Node("radar_world_calib")
{

    param_client_ = std::make_shared<rclcpp::SyncParametersClient>(this, "radar_station");
    while (!param_client_->wait_for_service(2s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for parameter service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for radar_station node ...");
        }
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&RadarWorldCalib::image_callback, this, std::placeholders::_1));

    cv::namedWindow("src", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("src", RadarWorldCalib::onMouse, this);

    RCLCPP_INFO(this->get_logger(), "RadarWorldCalib node has been created.");
}

void RadarWorldCalib::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat image_mat(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
    if (flip_image)
    {
        cv::rotate(image_mat, image_mat, cv::ROTATE_180);
    }
    cv::cvtColor(image_mat, image_mat, cv::COLOR_BGR2RGB);
    draw_x(image_mat, mouse_point_);

    for(size_t i = 0; i < image_points_.size(); i++)
    {
        draw_point(image_mat, image_points_[i], i + 1);
    }
    
    if(send_param){
        auto result = param_client_->set_parameters({
            rclcpp::Parameter("T_world2camera.Rotation.data", rvec_vector_),
            rclcpp::Parameter("T_world2camera.Translation.data", tvec_vector_)
        });

        for (const auto & res : result) {
            if (!res.successful) {
                RCLCPP_ERROR(this->get_logger(), "set param failed");
            } else {
                RCLCPP_INFO(this->get_logger(), "set param success");
            }
        }

        send_param = false;
    }

    cv::imshow("src", image_mat);
    cv::waitKey(1);
}
void RadarWorldCalib::onMouse(int event, int x, int y, int flags, void* userdata)
{
    mouse_point_ = cv::Point2f(x - 10, y - 10);
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        left_button_is_down_ = true;
    }
    if (event == cv::EVENT_LBUTTONUP)
    {
        if(point_count <= 4){
            point_count++;
            image_points_.push_back(mouse_point_);
        }
        left_button_is_down_ = false;
    }
    if(event == cv::EVENT_RBUTTONDOWN)
    {
        point_count = 0;
        image_points_.clear();
    }
    if(event == cv::EVENT_MBUTTONDOWN)
    {
        std::cout << "point_count: " << point_count << std::endl;
        if(point_count == 4){
            std::cout << "Start calibration........" << std::endl;
            cv::solvePnP(world_points_, image_points_, cameraMatrix, distCoeffs, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            std::cout << "R: " << R << std::endl;
            std::cout << "t: " << tvec << std::endl;

            rvec_vector_.clear();
            tvec_vector_.clear();

            rvec_vector_.push_back(R.at<double>(0, 0));
            rvec_vector_.push_back(R.at<double>(0, 1));
            rvec_vector_.push_back(R.at<double>(0, 2));

            rvec_vector_.push_back(R.at<double>(1, 0));
            rvec_vector_.push_back(R.at<double>(1, 1));
            rvec_vector_.push_back(R.at<double>(1, 2));

            rvec_vector_.push_back(R.at<double>(2, 0));
            rvec_vector_.push_back(R.at<double>(2, 1));
            rvec_vector_.push_back(R.at<double>(2, 2));

            tvec_vector_.push_back(tvec.at<double>(0));
            tvec_vector_.push_back(tvec.at<double>(1));
            tvec_vector_.push_back(tvec.at<double>(2));

            send_param = true;

        }else{
            std::cout << "Please add more points." << std::endl;
        }
    }
}

void RadarWorldCalib::draw_x(cv::Mat& image, cv::Point2d point)
{
    cv::line(image, cv::Point2f(point.x - 10, point.y - 10), cv::Point2f(point.x + 10, point.y + 10), cv::Scalar(0, 0, 255), 1);
    cv::line(image, cv::Point2f(point.x + 10, point.y - 10), cv::Point2f(point.x - 10, point.y + 10), cv::Scalar(0, 0, 255), 1);
    cv::putText(image, "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")", cv::Point2f(point.x + 10, point.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
}

void RadarWorldCalib::draw_point(cv::Mat& image, cv::Point2d point, int point_id)
{
    cv::circle(image, point, 4, cv::Scalar(255, 0, 0), 1);
    cv::putText(image,"P" + std::to_string(point_id) + "(" + std::to_string(point.x) + ", " + std::to_string(point.y) + ")", cv::Point2f(point.x + 10, point.y + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
}


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RadarWorldCalib>());
    rclcpp::shutdown();
    return 0;
}
