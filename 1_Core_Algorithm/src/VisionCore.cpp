#include "VisionCore.h"

VisionCore::VisionCore() {
    // 初始化面积阈值，防止把噪点当成目标
    red_area_threshold = 500;
    blue_area_threshold = 200;
}

VisionCore::~VisionCore() {}

PatrolResult VisionCore::processFrame(const cv::Mat& input_frame, cv::Mat& output_frame) {
    if (input_frame.empty()) return SAFE;

    // 拷贝一份用于画面标注，不破坏原图
    input_frame.copyTo(output_frame); 
    cv::Mat hsv_frame;
    cv::cvtColor(input_frame, hsv_frame, cv::COLOR_BGR2HSV);

    // -------------------------------------------------------------
    // Task 2: 检测红色违规插座
    // -------------------------------------------------------------
    cv::Mat mask_red1, mask_red2, mask_red;
    cv::inRange(hsv_frame, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask_red1);
    cv::inRange(hsv_frame, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask_red2);
    cv::bitwise_or(mask_red1, mask_red2, mask_red);

    int red_pixels = cv::countNonZero(mask_red);
    if (red_pixels > red_area_threshold) {
        // 在传出的画面上打上红色高能警告
        cv::putText(output_frame, "[WARNING] RED HAZARD DETECTED!", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        return HAZARD_RED_SOCKET;
    }

    // -------------------------------------------------------------
    // Task 3: 检测蓝色校园卡
    // -------------------------------------------------------------
    cv::Mat mask_blue;
    cv::inRange(hsv_frame, cv::Scalar(100, 150, 0), cv::Scalar(140, 255, 255), mask_blue);
    
    int blue_pixels = cv::countNonZero(mask_blue);
    if (blue_pixels > blue_area_threshold) {
        // 在传出的画面上打上蓝色失物提示
        cv::putText(output_frame, "[INFO] BLUE CARD FOUND!", cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255, 0, 0), 2);
        return LOST_BLUE_CARD;
    }

    return SAFE;
}