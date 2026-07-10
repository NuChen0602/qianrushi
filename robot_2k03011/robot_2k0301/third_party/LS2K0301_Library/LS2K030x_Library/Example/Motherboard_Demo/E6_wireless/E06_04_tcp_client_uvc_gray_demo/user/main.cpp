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
********************************************************************************************************************/

#include "zf_common_headfile.hpp"  // 引入逐飞科技LS2K030X开源库公共头文件，集成所有驱动/协议/工具函数

// *************************** 例程硬件连接说明 ***************************
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
// 
// *************************** 例程使用步骤说明 ***************************
// 1.根据硬件连接说明连接好模块，使用电源供电(下载器供电会导致模块电压不足)
//
// 2.查看电脑所连接的wifi，记录IP地址
//
// 3.在下方的代码区域中修改宏定义，SERVER_IP为电脑的IP地址，PORT为端口号
//
// 5.下载例程核心板中
//
// 6.打开逐飞助手，设置为TCP Server，设置端口，选择合适的本机地址后点击连接
//
//
// *************************** 例程测试说明 ***************************
// 1.当本机设备成功连接到目标后（例如电脑使用逐飞助手上位机进入TCP模式 然后本机连接到电脑的 IP 与端口）
//
// 2.本例程会先通过调试串口发送链接信息，然后向 目标 发送一段测试数据，
//   之后可以使用上位机发送数据给设备，设备收到之后会将数据发回给上位机
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查

// **************************** 代码区域 ****************************

// ====================== TCP网络配置宏定义 - 需手动修改适配 ======================
#define SERVER_IP     "192.168.2.49"   // TCP服务端的IP地址（电脑的局域网WiFi/有线IP）
#define PORT          8086             // TCP通信端口号，必须和逐飞助手上位机设置的端口一致

// ====================== 全局设备对象与变量定义 ======================
zf_driver_tcp_client tcp_client_dev;  // 定义TCP客户端设备对象，用于建立TCP连接和网络数据收发
zf_device_uvc uvc_dev;                // 定义UVC免驱摄像头设备对象，用于摄像头初始化/图像采集
uint8* rgay_image;                    // 灰度图像数据指针，指向摄像头采集到的灰度图像缓冲区首地址

// ====================== TCP收发数据包装函数 ======================
//-------------------------------------------------------------------------------------------------------------------
// 函数简介 TCP发送数据 全局包装函数
// 参数说明 buf 要发送的数据缓冲区指针, len 要发送的数据字节长度
// 返回参数 uint32 实际成功发送的字节数
// 使用示例 供seekfree_assistant_interface_init调用，无需手动调用
// 备注信息 封装tcp_client_dev.send_data成员函数，适配普通函数指针格式要求
//-------------------------------------------------------------------------------------------------------------------
uint32 tcp_send_wrap(const uint8 *buf, uint32 len)
{
    return tcp_client_dev.send_data(buf, len);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 TCP接收数据 全局包装函数
// 参数说明 buf 接收数据的缓冲区指针, len 最大可接收的字节长度
// 返回参数 uint32 实际成功接收的字节数
// 使用示例 供seekfree_assistant_interface_init调用，无需手动调用
// 备注信息 封装tcp_client_dev.read_data成员函数，适配普通函数指针格式要求
//-------------------------------------------------------------------------------------------------------------------
uint32 tcp_read_wrap( uint8 *buf, uint32 len)
{
    return tcp_client_dev.read_data(buf, len);
}

// ====================== 巡线边界信息配置相关宏定义与数组 ======================
// 边界信息包含类型配置，共5种模式，修改此宏可切换对应功能，当前配置为0：仅传图像 不传边界
// 0：不包含边界信息，仅向上位机发送摄像头原始灰度图像数据
// 1：包含三条边线信息，边线信息只含横轴X坐标，纵轴Y坐标由图像高度自动匹配，每行1个边界点
// 2：包含三条边线信息，边界信息只含纵轴Y坐标，横轴X坐标由图像宽度自动匹配，每列1个边界点(极少用)
// 3：包含三条边线信息，边界信息含完整XY轴坐标，可自定义任意坐标点，支持识别回弯道路，边界点数可自定义
// 4：无图像信息，仅包含三条边线X坐标，极大降低传输数据量，只传巡线数据，不传图像
#define INCLUDE_BOUNDARY_TYPE       0

#define BOUNDARY_NUM                (UVC_HEIGHT * 4 / 2)  // 边界点数量，适配TYPE3回弯场景，点数大于图像高度

// 边界坐标数组定义 - 对应不同的边界类型
uint8 xy_x1_boundary[BOUNDARY_NUM], xy_x2_boundary[BOUNDARY_NUM], xy_x3_boundary[BOUNDARY_NUM]; // TYPE3专用-X坐标
uint8 xy_y1_boundary[BOUNDARY_NUM], xy_y2_boundary[BOUNDARY_NUM], xy_y3_boundary[BOUNDARY_NUM]; // TYPE3专用-Y坐标
uint8 x1_boundary[UVC_HEIGHT], x2_boundary[UVC_HEIGHT], x3_boundary[UVC_HEIGHT];                 // TYPE1/4专用-X坐标
uint8 y1_boundary[UVC_WIDTH], y2_boundary[UVC_WIDTH], y3_boundary[UVC_WIDTH];                   // TYPE2专用-Y坐标

// 灰度图像数据拷贝缓冲区，二维数组存储，大小与摄像头采集分辨率一致，用于中转图像数据
uint8 image_copy[UVC_HEIGHT][UVC_WIDTH];

// **************************** 主函数 - 程序入口 ****************************
int main() 
{
    // 根据不同的边界类型，提前声明对应循环变量，节省内存，按需生效
#if(1 == INCLUDE_BOUNDARY_TYPE || 2 == INCLUDE_BOUNDARY_TYPE || 4 == INCLUDE_BOUNDARY_TYPE)
    int32 i = 0;
#elif(3 == INCLUDE_BOUNDARY_TYPE)
    int32 i = 0;
    int32 j = 0;
#endif

    // ====================== 第一步：初始化TCP客户端，建立网络连接 ======================
    if(tcp_client_dev.init(SERVER_IP, PORT) == 0)
    {
        printf("tcp_client ok\r\n");    // TCP连接成功，串口打印成功日志
    }
    else
    {
        printf("tcp_client error\r\n"); // TCP连接失败，串口打印错误日志
        return -1;                      // 连接失败直接退出程序，不执行后续流程
    }

    // ====================== 第二步：初始化逐飞助手通信接口 ======================
    // 实现上位机和下位机的TCP数据交互
    seekfree_assistant_interface_init(tcp_send_wrap, tcp_read_wrap);

    // ====================== 第三步：根据边界类型配置图像与巡线边界信息 ======================
    // 分支1：边界类型0 - 【默认启用】只发送灰度图像，不包含任何巡线边界信息
#if(0 == INCLUDE_BOUNDARY_TYPE)
    // 配置摄像头信息：指定摄像头类型为灰度摄像头MT9V03X、图像缓冲区地址、图像宽高
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);

    // 分支2：边界类型1 - 发送灰度图像 + 三条X轴巡线边界(每行1个点)
#elif(1 == INCLUDE_BOUNDARY_TYPE)
    // 给三条巡线边界X坐标赋值，生成渐变的边界轨迹模拟数据
    for(i = 0; i < UVC_HEIGHT; i++)
    {
        x1_boundary[i] = 50 - (50 - 20) * i / UVC_HEIGHT;
        x2_boundary[i] = UVC_WIDTH / 2;
        x3_boundary[i] = 70 + (148 - 70) * i / UVC_HEIGHT;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);
    // 配置X轴巡线边界信息，Y坐标由图像高度自动匹配
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, UVC_HEIGHT, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);

    // 分支3：边界类型2 - 发送灰度图像 + 三条Y轴巡线边界(每列1个点)，极少使用该模式
#elif(2 == INCLUDE_BOUNDARY_TYPE)
    for(i = 0; i < UVC_WIDTH; i++)
    {
        y1_boundary[i] = 50 - (50 - 20) * i / UVC_HEIGHT;
        y2_boundary[i] = UVC_WIDTH / 2;
        y3_boundary[i] = 78 + (78 - 58) * i / UVC_HEIGHT;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);
    // 配置Y轴巡线边界信息，X坐标由图像宽度自动匹配
    seekfree_assistant_camera_boundary_config(Y_BOUNDARY, UVC_WIDTH, NULL, NULL ,NULL, y1_boundary, y2_boundary, y3_boundary);

    // 分支4：边界类型3 - 发送灰度图像 + 完整XY轴巡线边界，支持回弯道路识别，功能最强
#elif(3 == INCLUDE_BOUNDARY_TYPE)
    j = 0;
    // 生成直线段的边界坐标
    for(i = UVC_HEIGHT - 1; i >= UVC_HEIGHT / 2; i--)
    {
        xy_x1_boundary[j] = 34; xy_y1_boundary[j] = i;
        xy_x2_boundary[j] = 47; xy_y2_boundary[j] = i;
        xy_x3_boundary[j] = 60; xy_y3_boundary[j] = i;
        j++;
    }
    // 生成直线过渡到弯道的边界坐标
    for(i = UVC_HEIGHT / 2 - 1; i >= 0; i--)
    {
        xy_x1_boundary[j] = 34 + (UVC_HEIGHT / 2 - i) * (UVC_WIDTH / 2 - 34) / (UVC_HEIGHT / 2);
        xy_y1_boundary[j] = i;
        xy_x2_boundary[j] = 47 + (UVC_HEIGHT / 2 - i) * (UVC_WIDTH / 2 - 47) / (UVC_HEIGHT / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;
        xy_x3_boundary[j] = 60 + (UVC_HEIGHT / 2 - i) * (UVC_WIDTH / 2 - 60) / (UVC_HEIGHT / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    // 生成纯回弯段的边界坐标
    for(i = 0; i < UVC_HEIGHT / 2; i++)
    {
        xy_x1_boundary[j] = UVC_WIDTH / 2 + i * (138 - UVC_WIDTH / 2) / (UVC_HEIGHT / 2);
        xy_y1_boundary[j] = i;
        xy_x2_boundary[j] = UVC_WIDTH / 2 + i * (133 - UVC_WIDTH / 2) / (UVC_HEIGHT / 2);
        xy_y2_boundary[j] = 15 + i * 3 / 4;
        xy_x3_boundary[j] = UVC_WIDTH / 2 + i * (128 - UVC_WIDTH / 2) / (UVC_HEIGHT / 2);
        xy_y3_boundary[j] = 30 + i / 2;
        j++;
    }
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], UVC_WIDTH, UVC_HEIGHT);
    // 配置完整XY轴巡线边界信息
    seekfree_assistant_camera_boundary_config(XY_BOUNDARY, BOUNDARY_NUM, xy_x1_boundary, xy_x2_boundary, xy_x3_boundary, xy_y1_boundary, xy_y2_boundary, xy_y3_boundary);

    // 分支5：边界类型4 - 只发送三条X轴巡线边界，不发送灰度图像，极大节省带宽
#elif(4 == INCLUDE_BOUNDARY_TYPE)
    for(i = 0; i < UVC_HEIGHT; i++)
    {
        x1_boundary[i] = 70 - (70 - 20) * i / UVC_HEIGHT;
        x2_boundary[i] = UVC_WIDTH / 2;
        x3_boundary[i] = 80 + (159 - 80) * i / UVC_HEIGHT;
    }
    // 图像缓冲区传NULL，表示不发送图像数据
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, NULL, UVC_WIDTH, UVC_HEIGHT);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, UVC_HEIGHT, x1_boundary, x2_boundary, x3_boundary, NULL, NULL ,NULL);
#endif

    // ====================== 第四步：初始化UVC免驱摄像头 ======================
    if(uvc_dev.init(UVC_PATH) < 0)
    {
        return -1;  // 摄像头初始化失败，直接退出程序
    }

    // ====================== 主循环：持续采集灰度图像 + TCP发送 ======================
    while (true) 
    {
        // 阻塞式等待摄像头完成新一帧图像的采集刷新，返回<0表示采集失败/异常
        if(uvc_dev.wait_image_refresh() < 0)
        {
            exit(0);   // 摄像头采集异常，直接退出程序，防止卡死
        }

        // 获取摄像头采集到的灰度图像数据缓冲区首地址
        rgay_image = uvc_dev.get_gray_image_ptr();
        
        // 把灰度图像数据拷贝到中转缓冲区，灰度图每个像素占1字节，无需格式转换
        memcpy(image_copy[0], rgay_image, UVC_WIDTH * UVC_HEIGHT);

        // 核心发送函数：将灰度图像/边界信息 通过TCP网络发送到逐飞助手上位机
        seekfree_assistant_camera_send();
    }

    return 0;  // 死循环不会执行到此行，程序正常退出返回值
}

// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：终端提示未找到xxx文件
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核。
// 
// 问题2：屏幕不显示
//      屏幕的初始化，是在开机的时候完成的，所以需要开启久久派之前插入屏幕
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
//
// 问题3：摄像头没找到
//      重新插入UVC摄像头，摄像头支持热插拔。