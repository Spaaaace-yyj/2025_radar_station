#ifndef ONNX_BOX_HPP_
#define ONNX_BOX_HPP_

#include <opencv4/opencv2/opencv.hpp>
#include <iostream>

class OnnxBox{
public:
    OnnxBox() = default;
    ~OnnxBox() = default;

    void draw(cv::Mat& dst);

    void print_info();

public:
    cv::Point2f P1;
    cv::Point2f P2;
public:
    float width;
    float height;
    cv::Point2f center;
public:
    float conf;
    int class_id = -1;
};


#endif