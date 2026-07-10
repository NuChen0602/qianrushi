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
* 文件名称          main.cpp
* 公司名称          成都逐飞科技有限公司
* 适用平台          LS2K030X
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者           备注
* 2026-01-08        逐飞科技       新增帧率统计main函数
********************************************************************************************************************/

// 引入必要头文件

#include "zf_common_headfile.hpp"  // 引入逐飞科技LS2K030X开源库公共头文件，集成所有基础驱动/协议接口

// ====================== 配置项 ======================
#define UVC_PATH       "/dev/video0"  // 摄像头设备节点（根据实际情况修改）

#define PRINT_INTERVAL 1000           // 帧率打印间隔（毫秒），即每秒打印一次

// 全局变量：用于信号处理，控制程序退出
bool g_running = true;

// 信号处理函数：捕获Ctrl+C，优雅退出
void signal_handler(int sig)
{
    if (sig == SIGINT)
    {
        std::cout << "\n收到退出信号，程序即将停止..." << std::endl;
        g_running = false;
    }
}

// ====================== 主函数 ======================
int main()
{

    std::cout << "UVC摄像头帧率统计程序启动..." << std::endl;
    std::cout << "摄像头设备：" << UVC_PATH << std::endl;
    std::cout << "分辨率：" << UVC_WIDTH << "x" << UVC_HEIGHT << std::endl;
    std::cout << "期望帧率：" << UVC_FPS << " FPS" << std::endl;
    std::cout << "按 Ctrl+C 退出程序\n" << std::endl;

    // 2. 初始化UVC摄像头对象
    zf_device_uvc uvc_dev;
    if (uvc_dev.init(UVC_PATH) < 0)
    {
        std::cerr << "摄像头初始化失败！请检查设备节点或摄像头连接" << std::endl;
        return -1;
    }

    // 3. 帧率统计变量初始化
    int frame_count = 0;                                      // 累计帧数
    auto start_time = std::chrono::steady_clock::now();       // 统计起始时间
    auto last_print_time = start_time;                        // 上一次打印帧率的时间

    // 4. 主循环：采集图像 + 统计帧率
    while (g_running)
    {
        // 阻塞式等待摄像头刷新一帧图像
        int8_t ret = uvc_dev.wait_image_refresh();
        if (ret < 0)
        {
            std::cerr << "图像采集失败，跳过当前帧" << std::endl;
            continue;
        }

        // 读取图像数据（仅为了模拟实际使用场景，可根据需求选择灰度/彩色）
        // uvc_dev.get_rgb_image_ptr(); // 或 get_gray_image_ptr()

        // 帧数+1
        frame_count++;

        // 5. 每秒统计一次帧率
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double, std::milli> elapsed = current_time - last_print_time;
        
        if (elapsed.count() >= PRINT_INTERVAL)
        {
            // 计算实际帧率：帧数 / 耗时（秒）
            double fps = frame_count / (elapsed.count() / 1000.0);
            
            // 输出帧率信息
            std::cout << "实时帧率：" << fps << " FPS | 累计帧数：" << frame_count << std::endl;
            
            // 重置统计变量
            frame_count = 0;
            last_print_time = current_time;
        }
    }

    // 6. 程序退出：释放资源
    std::cout << "\n程序退出，释放摄像头资源..." << std::endl;
    // 析构函数会自动释放摄像头，此处仅作提示

    return 0;
}