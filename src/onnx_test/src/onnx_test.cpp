#include "../include/onnx_test/onnx_test.hpp"

OnnxTest::OnnxTest() : Node("onnx_test")
{
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
        "/image_raw", 10, std::bind(&OnnxTest::image_callback, this, std::placeholders::_1));
    load_model();
}

void OnnxTest::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat frame_;
    cv::Mat image_mat_(msg->height, msg->width, CV_8UC3, const_cast<unsigned char*>(msg->data.data()), msg->step);
    cv::cvtColor(image_mat_, image_mat_, cv::COLOR_BGR2RGB);
    cv::rotate(image_mat_, frame_, cv::ROTATE_180);

    cv::Mat resize;
    cv::resize(frame_, resize, cv::Size(640, 640));

    cv::Mat blob_car = cv::dnn::blobFromImage(resize, 1.0 / 255.0, cv::Size(640, 640), cv::Scalar(0, 0, 0), true, false);
    car_net.setInput(blob_car);
    std::vector<cv::Mat> outs_car;
    std::vector<std::string> outNames = { "output0" };
    car_net.forward(outs_car, outNames);

    process_tensor(outs_car[0], frame_);
    
    cv::imshow("image", frame_);
    cv::waitKey(1);
}

void OnnxTest::process_tensor(cv::Mat& outs, cv::Mat& frame){
    int rows = outs.size[1];
    int dimensions = outs.size[2];
    bool yolov8 = false;
    if (dimensions > rows){
        yolov8 = true;
        rows = outs.size[2];
        dimensions = outs.size[1];

        outs = outs.reshape(1, dimensions);
        cv::transpose(outs, outs);
    }

    float *data = (float *)outs.data;

    for(int i = 0; i < rows; i++){
        if(yolov8){
            std::cout << "yolov8" << std::endl;

            float max_score = 0;
            int max_class_id = 0;
            float *classes_scores = data + 4;
            for(int j = 0; j < dimensions - 4; j++){
                if(classes_scores[j] > max_score){
                    max_score = classes_scores[j];
                    max_class_id = j;
                }
                if(max_score > 0.5){
                    float x = (data[0] / 640) * frame.cols;
                    float y = (data[1] / 640) * frame.rows;
                    float w = (data[2] / 640) * frame.cols;
                    float h = (data[3] / 640) * frame.rows;
                    cv::putText(frame, std::to_string(max_class_id), cv::Point(x - w / 2, y - h / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    cv::rectangle(frame, cv::Point(x - w / 2, y - h / 2), cv::Point(x + w / 2, y + h / 2), cv::Scalar(0, 255, 0), 2);
                }
            }
        }else{
            float obj_score = data[4];
            if(obj_score > 0.2){
                std::cout << "yolov5" << std::endl;
                float *classes_scores = data + 5;
                float max_score = 0;
                int max_class_id = 0;
                for(int j = 0; j < dimensions - 5; j++){
                    if(classes_scores[j] > max_score){
                        max_score = classes_scores[j];
                        max_class_id = j;
                    }
                }
                if(max_score > 0.6){
                    float x = (data[0] / 640) * frame.cols;
                    float y = (data[1] / 640) * frame.rows;
                    float w = (data[2] / 640) * frame.cols;
                    float h = (data[3] / 640) * frame.rows;
                    cv::putText(frame, std::to_string(max_class_id), cv::Point(x - w / 2, y - h / 2), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    cv::rectangle(frame, cv::Point(x - w / 2, y - h / 2), cv::Point(x + w / 2, y + h / 2), cv::Scalar(0, 255, 0), 2);
                }
            }
        }
        data += dimensions;
    }
}

void OnnxTest::load_model(){
    car_net = cv::dnn::readNetFromONNX(car_onnx_path);
    armor_net = cv::dnn::readNetFromONNX(armor_onnx_path);

    car_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    car_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);
    armor_net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA);
    armor_net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA);

    RCLCPP_INFO(get_logger(), "load model success");

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OnnxTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}