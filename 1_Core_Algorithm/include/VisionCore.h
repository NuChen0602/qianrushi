#ifndef VISION_CORE_H
#define VISION_CORE_H

#include <opencv2/opencv.hpp>

// 定义巡检状态的枚举值
enum PatrolResult {
    SAFE = 0,
    HAZARD_RED_SOCKET = 1,
    LOST_BLUE_CARD = 2
};

class VisionCore {
public:
    VisionCore();
    ~VisionCore();

    // 核心处理函数：输入原始图像，输出识别状态，并传出带有标注框的可视化图像
    PatrolResult processFrame(const cv::Mat& input_frame, cv::Mat& output_frame);

private:
    // 算法参数（面积阈值）
    int red_area_threshold;
    int blue_area_threshold;
};

#endif