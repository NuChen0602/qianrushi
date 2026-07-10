/*********************************************************************************************************************
* LS2K030X Opensourec Library 即（LS2K030X 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是LS2K030X 开源库的一部分
*
* LS2K030X 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K030X
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2025-12-27        大W            first version
* 2026-03-06        优化           添加分级可控日志输出
********************************************************************************************************************/


#include "zf_common_headfile.hpp"
using namespace cv;
zf_device_uvc uvc_dev; 


// *************************** 例程硬件连接说明 ***************************
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
// 
// 
// *************************** 例程测试说明 ***************************
// 1.将2K30x核心板插到主板上面 主板使用电池供电 下载本例程
// 
// 2.运行后可以看到模型输出返回的类别信息和置信度以及模型推理所使用的时间
//
// 3.本例程仅为演示如何将摄像头数据传入模型 无法保证模型输出结果与理想一致
//
// ======================== 可配置参数宏定义 ========================
// 1. 模型相关配置
#define MODEL_INPUT_WIDTH         40      // 模型输入图像宽度（根据实际模型调整）
#define MODEL_INPUT_HEIGHT        40      // 模型输入图像高度（根据实际模型调整）
#define MODEL_INPUT_CHANNEL       3       // 模型输入图像通道数（RGB=3）
#define MODEL_OUTPUT_CLASS_NUM    3       // 模型输出类别数（根据实际任务调整）
#define MODEL_INPUT_SIZE          (MODEL_INPUT_WIDTH * MODEL_INPUT_HEIGHT * MODEL_INPUT_CHANNEL)  // 输入总元素数
#define TFLITE_OP_RESOLVER_MAX_NUM 20     // 算子解析器最大支持算子数
#define TENSOR_ARENA_SIZE         (128 * 1024)  // 张量空间大小（单位：字节，根据模型大小调整）
const char* class_labels[] = {"materials","traffic","weapon"}; //需要与train.py提示顺序一致

// 2. 日志控制配置
#define LOG_PRINT_FREQUENCY       10      // 每N帧打印一次推理结果（降低刷屏频率）
#define ENABLE_FIRST_FRAME_DEBUG  true    // 打印第一帧的完整预处理日志（调试用）

// ======================== 日志回调函数 ========================
void debug_log_printf_callback(const char* s) 
{
    if (s == NULL)  return;
    printf("%s", s);
}

// ======================== 主函数 ========================
int main(int, char**) 
{
    // 重定向TFLM的MicroPrintf日志输出
    RegisterDebugLogCallback(debug_log_printf_callback);

    // tflite库初始化
    tflite::InitializeTarget();

    // 导入模型
    // 传入模型文件生成的.h文件当中的数组名
    const tflite::Model* model = ::tflite::GetModel(loong_cnn_model_simple_tflite);
    TFLITE_CHECK_EQ(model->version(), TFLITE_SCHEMA_VERSION);

    // 构建算子解析器（使用宏定义指定最大算子数）
    tflite::MicroMutableOpResolver<TFLITE_OP_RESOLVER_MAX_NUM> resolver;

    /* 添加所有可能会用到的算子 */
	TF_LITE_ENSURE_STATUS(resolver.AddConv2D());
	TF_LITE_ENSURE_STATUS(resolver.AddDepthwiseConv2D());
	TF_LITE_ENSURE_STATUS(resolver.AddMaxPool2D());
	TF_LITE_ENSURE_STATUS(resolver.AddFullyConnected());
	TF_LITE_ENSURE_STATUS(resolver.AddRelu6());
	TF_LITE_ENSURE_STATUS(resolver.AddSoftmax());
	TF_LITE_ENSURE_STATUS(resolver.AddReshape());
	TF_LITE_ENSURE_STATUS(resolver.AddShape());
	TF_LITE_ENSURE_STATUS(resolver.AddQuantize());
	TF_LITE_ENSURE_STATUS(resolver.AddDequantize());
	TF_LITE_ENSURE_STATUS(resolver.AddCast());
	TF_LITE_ENSURE_STATUS(resolver.AddSqueeze());
	TF_LITE_ENSURE_STATUS(resolver.AddExpandDims());
	TF_LITE_ENSURE_STATUS(resolver.AddConcatenation());
	TF_LITE_ENSURE_STATUS(resolver.AddTranspose());
	TF_LITE_ENSURE_STATUS(resolver.AddStridedSlice());
	TF_LITE_ENSURE_STATUS(resolver.AddPack());
    TF_LITE_ENSURE_STATUS(resolver.AddLogistic());
    TF_LITE_ENSURE_STATUS(resolver.AddMean());
    TF_LITE_ENSURE_STATUS(resolver.AddAdd());

    // 创建模型解析器并分配张量空间（使用宏定义指定张量空间大小）
    uint8_t tensor_arena[TENSOR_ARENA_SIZE];
    tflite::MicroInterpreter interpreter(model, resolver, tensor_arena, TENSOR_ARENA_SIZE);
    interpreter.AllocateTensors();

    //摄像头初始化
    printf("===== 程序初始化 =====\n");
    if(uvc_dev.init(UVC_PATH) < 0)
    {
        printf("❌ 摄像头初始化失败！\n");
        return -1;  // 摄像头初始化失败，直接退出程序
    }
    printf("✅ 摄像头初始化成功\n");

    //cv图像矩阵初始化
    Mat src_img,float_img,resized_img;
    
    // 日志控制变量
    int frame_count = 0;          // 帧计数器
    bool first_frame_logged = false;  // 第一帧日志是否已打印

    printf("===== 开始实时推理 =====\n");
    while(1)
    {
        frame_count++;  // 帧计数+1

        //摄像头采集数据
        if(uvc_dev.wait_image_refresh() < 0)
        {
            printf("❌ 第%d帧：摄像头采集异常，程序退出！\n", frame_count);
            exit(0);   // 摄像头采集异常，退出程序，防止程序卡死
        }
        //获取摄像头采集原始数据
        src_img = uvc_dev.get_frame_mjpg();

        if(src_img.empty())
        {
            printf("❌ 第%d帧：获取图像数据为空！\n", frame_count);
            continue;  // 跳过空帧，不退出程序
        }

        // ======================== 图像预处理 ========================
        // 1. 先缩放图像到模型要求的输入尺寸
        resize(src_img, resized_img, Size(MODEL_INPUT_WIDTH, MODEL_INPUT_HEIGHT), INTER_LINEAR);

        // 2. 类型转换（转为32位浮点型，可选归一化）
        resized_img.convertTo(float_img, CV_32FC3, 1.0);  // 训练时归一化则用1.0/255.0，否则用1.0

        // 3. 色彩空间转换（BGR→RGB，适配模型输入）
        cvtColor(float_img, float_img, cv::COLOR_BGR2RGB);

        // 4. 确保内存连续，避免拷贝错误（可选）
        Mat continuous_float_img = float_img.isContinuous() ? float_img : float_img.clone();
        float* model_input_ptr = continuous_float_img.ptr<float>(0);

        // ===== 第一帧打印完整预处理日志（仅打印一次）=====
        if(ENABLE_FIRST_FRAME_DEBUG && !first_frame_logged)
        {
            printf("\n===== 第1帧 完整预处理日志 =====\n");
            // 原始图像信息
            printf("1. 原始图像：%d x %d (宽x高)，通道数：%d\n", src_img.cols, src_img.rows, src_img.channels());
            // 缩放后图像信息
            printf("2. 缩放后图像：%d x %d (宽x高)，通道数：%d\n", resized_img.cols, resized_img.rows, resized_img.channels());
            // 内存连续性检查
            printf("3. 浮点图像内存连续：%s\n", float_img.isContinuous() ? "是" : "否");
            printf("4. 模型输入尺寸匹配：%s\n", 
                   (continuous_float_img.total() * continuous_float_img.elemSize()) == (MODEL_INPUT_SIZE * sizeof(float)) 
                   ? "✅ 匹配" : "❌ 不匹配");
            printf("=================================\n");
            first_frame_logged = true;  // 标记第一帧日志已打印
        }

        // ======================== 模型输入 ========================
        float* input_data = interpreter.input(0)->data.f;
        memcpy(input_data, model_input_ptr, MODEL_INPUT_SIZE * sizeof(float));

        // ======================== 模型推理（计时） ========================
        auto start_time = std::chrono::high_resolution_clock::now();
        interpreter.Invoke();
        auto end_time = std::chrono::high_resolution_clock::now();

        // 计算推理耗时
        auto time_cnt = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time).count();

        // ======================== 模型输出解析 ========================
        float* output_data = interpreter.output(0)->data.f;
        float max_v = 0;
        int max_i = 0;
        
        // 查找置信度最高的类及其索引
        for (int n = 0; n < MODEL_OUTPUT_CLASS_NUM; n++) 
        {
            float v = output_data[n];
            if (v > max_v) {
                max_v = v;
                max_i = n;
            }
        }

        // ===== 可控频率打印推理结果 =====
        if(frame_count % LOG_PRINT_FREQUENCY == 0)
        {
            // 打印简洁的推理结果（每N帧一次）
            printf("[第%d帧] 预测类别：%s | 置信度：%.4f | 推理耗时：%ld us\n",
                   frame_count, class_labels[max_i], max_v, time_cnt);
            
            // 可选：打印所有类别的置信度（调试用，按需开启）
            // printf("        详细置信度：%s=%.4f, %s=%.4f, %s=%.4f\n",
            //        class_labels[0], output_data[0],
            //        class_labels[1], output_data[1],
            //        class_labels[2], output_data[2]);
        }

    }

    return 0; 
}