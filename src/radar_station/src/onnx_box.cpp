#include "../include/radar_station/onnx_box.hpp"

void OnnxBox::draw(cv::Mat& dst){
    if(class_id != -1){
        cv::rectangle(dst, P2, P1, cv::Scalar(0, 255, 0), 2);
        cv::putText(dst, "ID: " + std::to_string(class_id), P1, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::putText(dst, std::to_string(conf), P2, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::circle(dst, center, 5, cv::Scalar(0, 0, 255), 2);
    }else{
        cv::rectangle(dst, P2, P1, cv::Scalar(0, 255, 0), 2);
        cv::putText(dst, "ID: ???", P1, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::putText(dst, std::to_string(conf), P2, cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);
        cv::circle(dst, center, 5, cv::Scalar(0, 0, 255), 2);
    }
    
}

void OnnxBox::print_info(){
    std::cout << "class_id: " << class_id << std::endl;
    std::cout << "conf: " << conf << std::endl;
    std::cout << "P1: " << P1 << std::endl;
    std::cout << "P2: " << P2 << std::endl;
}
