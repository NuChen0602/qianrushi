
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

#include "zf_common_headfile.hpp"

// *************************** 例程硬件连接说明 ***************************
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
// 
// 接入 IMU963RA或IMU660RA或IMU660RB
// 接入 IMU963RA或IMU660RA或IMU660RB
// 接入 IMU963RA或IMU660RA或IMU660RB
// 改例程序自动判断是哪一个姿态传感器
//      模块管脚             单片机管脚
//      SCL/SPC             查看 seekfree_2K30x_coreboard.dts 文件中 &spi1 节点定义 默认 GPIO60
//      SDA/DSI             查看 seekfree_2K30x_coreboard.dts 文件中 &spi1 节点定义 默认 GPIO62
//      SA0/SDO             查看 seekfree_2K30x_coreboard.dts 文件中 &spi1 节点定义 默认 GPIO61
//      CS                  查看 seekfree_2K30x_coreboard.dts 文件中 &spi1 节点定义 默认 GPIO63
//      GND                 电源地 GND
//      3V3                 电源 3V3
// *************************** 例程测试说明 ***************************
// 1.先将姿态传感器插入主板，运行代码。
// 
// 2.打开终端，使用115200波特率
//
// 4.终端中看到
//      imu_acc_x  = xxx.
//      ...
// 5.移动旋转 姿态传感器 就会看到数值变化
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查
// **************************** 代码区域 ****************************

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 全局变量-定义PIT定时器设备对象
// 参数说明 无
// 返回参数 无
// 使用示例 pit_timer.init_ms(10, pit_callback);
// 备注信息 用于创建周期中断定时器，实现IMU数据定时采集
//-------------------------------------------------------------------------------------------------------------------
zf_driver_pit pit_timer;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 全局变量-IMU六轴/九轴传感器原始数据存储变量
// 参数说明 无
// 返回参数 无
// 使用示例 直接赋值与读取，均为int16整型原始采集值
// 备注信息 acc:三轴加速度  gyro:三轴陀螺仪  mag:三轴磁力计(仅963RA有数据)
//-------------------------------------------------------------------------------------------------------------------
int16 imu_acc_x,imu_acc_y,imu_acc_z;
int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16 imu_mag_x,imu_mag_y,imu_mag_z;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 全局变量-定义IMU传感器设备对象
// 参数说明 无
// 返回参数 无
// 使用示例 imu_dev.init(); imu_dev.get_acc_x();
// 备注信息 封装IMU初始化/数据读取/设备类型识别等所有相关属性和成员函数
//-------------------------------------------------------------------------------------------------------------------
zf_device_imu imu_dev;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 PIT定时器10ms周期中断回调函数
// 参数说明 无
// 返回参数 无
// 使用示例 pit_timer.init_ms(10, pit_callback); 注册该回调函数
// 备注信息 定时器中断内执行，只做精简的IMU数据读取赋值，不做耗时操作；自动兼容660RA/660RB/963RA型号
//-------------------------------------------------------------------------------------------------------------------
void pit_callback()
{
    // 读取六轴传感器(IMU660RA/IMU660RB)数据:加速度+陀螺仪
    if(DEV_IMU660RA == imu_dev.imu_type || DEV_IMU660RB == imu_dev.imu_type)
    {
        imu_acc_x = imu_dev.get_acc_x();
        imu_acc_y = imu_dev.get_acc_y();
        imu_acc_z = imu_dev.get_acc_z();

        imu_gyro_x = imu_dev.get_gyro_x();
        imu_gyro_y = imu_dev.get_gyro_y();
        imu_gyro_z = imu_dev.get_gyro_z();
    }
    // 读取九轴传感器(IMU963RA)数据:加速度+陀螺仪+磁力计
    else if(DEV_IMU963RA == imu_dev.imu_type)
    {
        imu_acc_x = imu_dev.get_acc_x();
        imu_acc_y = imu_dev.get_acc_y();
        imu_acc_z = imu_dev.get_acc_z();

        imu_gyro_x = imu_dev.get_gyro_x();
        imu_gyro_y = imu_dev.get_gyro_y();
        imu_gyro_z = imu_dev.get_gyro_z();

        imu_mag_x = imu_dev.get_mag_x();
        imu_mag_y = imu_dev.get_mag_y();
        imu_mag_z = imu_dev.get_mag_z();
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介 程序入口主函数
// 参数说明 int 无实际传入参数，char** 无实际传入参数，标准工程模板格式
// 返回参数 int 程序运行状态，正常循环不返回，识别不到IMU返回-1，正常退出返回0
// 使用示例 程序上电自动执行，无需手动调用
// 备注信息 初始化硬件设备→检测设备状态→创建定时器→主循环打印数据，为程序总流程入口
//-------------------------------------------------------------------------------------------------------------------
int main(int, char**) 
{
    // IMU传感器初始化，自动检测挂载的IMU型号并完成底层配置
    imu_dev.init();

    // 识别IMU传感器型号并打印串口信息
    if(DEV_IMU660RA == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU660RA\r\n");
    }
    else if(DEV_IMU660RB == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU660RB\r\n");
    }
    else if(DEV_IMU963RA == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU963RA\r\n");
    }
    else
    {
        // 未检测到有效IMU设备，打印错误信息并退出程序
        printf("NO FIND IMU DEV\r\n");
        return -1;
    }

    // 初始化PIT定时器，周期10ms，定时到执行pit_callback回调函数
    pit_timer.init_ms(10, pit_callback);

    // 主循环
    while(1)
    {
        // 六轴传感器数据打印：加速度+陀螺仪
        if(DEV_IMU660RA == imu_dev.imu_type || DEV_IMU660RB == imu_dev.imu_type)
        {
            printf("imu_acc_x  = %d\r\n", imu_acc_x);
            printf("imu_acc_y  = %d\r\n", imu_acc_y);
            printf("imu_acc_z  = %d\r\n", imu_acc_z);
    
            printf("imu_gyro_x = %d\r\n", imu_gyro_x);
            printf("imu_gyro_y = %d\r\n", imu_gyro_y);
            printf("imu_gyro_z = %d\r\n", imu_gyro_z);
        }
        // 九轴传感器数据打印：加速度+陀螺仪+磁力计
        else if(DEV_IMU963RA == imu_dev.imu_type)
        {
            printf("imu_acc_x  = %d\r\n", imu_acc_x);
            printf("imu_acc_y  = %d\r\n", imu_acc_y);
            printf("imu_acc_z  = %d\r\n", imu_acc_z);
    
            printf("imu_gyro_x = %d\r\n", imu_gyro_x);
            printf("imu_gyro_y = %d\r\n", imu_gyro_y);
            printf("imu_gyro_z = %d\r\n", imu_gyro_z);
    
            printf("imu_mag_x = %d\r\n", imu_mag_x);
            printf("imu_mag_y = %d\r\n", imu_mag_y);
            printf("imu_mag_z = %d\r\n", imu_mag_z);
        }

        // 延时100ms，降低串口打印频率，防止数据刷屏过快
        system_delay_ms(100);
    }
    return 0;
}
// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：终端提示未找到xxx文件
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到