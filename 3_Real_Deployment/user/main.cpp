/*********************************************************************************************************************
* LS2K0300 Opensourec Library 即（LS2K0300 开源库）
* Copyright (c) 2022 SEEKFREE 逐飞科技
* * 【修改记录】
* 2026 - Fangzhou - 接入 OpenCV 零拷贝视觉链路，新增目标识别预处理算子
********************************************************************************************************************/

#include "zf_common_headfile.h"
#include <opencv2/opencv.hpp>  // 【核心修改 1】引入 OpenCV 头文件

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    printf("程序异常退出，执行清理操作\n");
    // 这里未来可以加入关闭电机、舵机复位的安全保护代码
}

int main(int, char**) 
{
    // 注册清理函数和信号处理
    atexit(cleanup);
    signal(SIGINT, sigint_handler);

    // 初始化 2 寸 IPS 屏幕
    ips200_init("/dev/fb0");

    // 初始化 UVC 摄像头
    if(uvc_camera_init("/dev/video0") < 0)
    {
        printf("摄像头初始化失败，请检查 USB 连接！\n");
        return -1;
    }
    
    printf("底层硬件初始化完毕，开启 OpenCV 视觉智巡引擎...\n");

    while(1)
    {
        // 1. 阻塞式等待底层 V4L2 硬件图像刷新
        if(wait_image_refresh() < 0)
        {
            printf("警告：摄像头断开连接或掉帧！\n");
            exit(0);
        }

        // ===================================================================
        // 【核心视觉手术：Zero-Copy 零拷贝接入 OpenCV】
        // 学长底层的 rgay_image 是一个单纯的内存指针。
        // 我们不拷贝它，直接用 cv::Mat 给这块内存套上一个“外壳”。
        // 这种操作不消耗任何 CPU 算力，完美榨干 1.0GHz 芯片的性能。
        // ===================================================================
        cv::Mat raw_frame(UVC_HEIGHT, UVC_WIDTH, CV_8UC1, rgay_image);

        // ===================================================================
        // 【任务预留区：插座与失物识别前处理】
        // 考虑到算力，我们这里先用 Otsu（大津法）自适应阈值进行二值化处理。
        // 这能极大地凸显出墙面插座的矩形轮廓和书籍/校园卡的硬边缘。
        // ===================================================================
        cv::Mat processed_frame;
        cv::threshold(raw_frame, processed_frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

        // ===================================================================
        // 【逻辑区】
        // 未来你的 NCNN 推理模型（找插座）和 MQTT 发送代码（云边协同）就写在这里
        // ===================================================================


        // 2. 将 OpenCV 处理后的结果，直接推送到 2 寸屏幕上实时显示！
        // processed_frame.data 就是处理后的图像内存首地址
        ips200_show_gray_image(0, 0, processed_frame.data, UVC_WIDTH, UVC_HEIGHT);
    }
}