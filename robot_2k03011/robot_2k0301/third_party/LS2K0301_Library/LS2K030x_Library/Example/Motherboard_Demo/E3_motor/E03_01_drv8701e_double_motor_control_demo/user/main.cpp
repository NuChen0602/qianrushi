
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
// 
// 使用核心板派直接接线进行测试
//      模块管脚            单片机管脚
//      MOTOR1_DIR          GPIO73
//      MOTOR1_PWM          GPIO86
//      GND                 GND   
//      MOTOR2_DIR          GPIO76
//      MOTOR2_PWM          GPIO87
//      GND                 GND
//      接线端子 +          电池正极
//      接线端子 -          电池负极
// 
// 使用学习主板进行测试
//      将模块的电源接线端子与主板的驱动供电端子连接
//      将模块的信号接口使用配套灰排线与主板电机信号接口连接 请注意接线方向 不确定方向就是用万用表确认一下 引脚参考上方核心板连接
//      将主板与供电电池正确连接
// 
// *************************** 例程测试说明 ***************************
// 1.核心板烧录完成本例程 主板电池供电
// 
// 2.如果接了电机 可以看到电机周期正反转
// 
// 3.如果没有接电机 使用万用表可以在驱动电机输出端子上测量到输出电压变化
// 
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查
// **************************** 代码区域 ****************************

#define PWM_1_PATH        ZF_PWM_MOTOR_1
#define DIR_1_PATH        ZF_GPIO_MOTOR_1

#define PWM_2_PATH        ZF_PWM_MOTOR_2
#define DIR_2_PATH        ZF_GPIO_MOTOR_2
zf_driver_pit pit_timer;

struct pwm_info drv8701e_pwm_1_info;
struct pwm_info drv8701e_pwm_2_info;


zf_driver_gpio  drv8701e_dir_1(DIR_1_PATH, O_RDWR);
zf_driver_gpio  drv8701e_dir_2(DIR_2_PATH, O_RDWR);
zf_driver_pwm   drv8701e_pwm_1(PWM_1_PATH);
zf_driver_pwm   drv8701e_pwm_2(PWM_2_PATH);

int8 duty = 0;
bool dir = true;

// 在设备树中，设置的10000。如果要修改，需要与设备树对应。
#define MOTOR1_PWM_DUTY_MAX    (drv8701e_pwm_1_info.duty_max)       
// 在设备树中，设置的10000。如果要修改，需要与设备树对应。 
#define MOTOR2_PWM_DUTY_MAX    (drv8701e_pwm_2_info.duty_max)        

#define MAX_DUTY        (30 )   // 最大 MAX_DUTY% 占空比

void sigint_handler(int signum) 
{
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}

void cleanup()
{
    // 需要先停止定时器线程，后面才能稳定关闭电机，电调，舵机等
    pit_timer.stop();
    printf("程序异常退出，执行清理操作\n");
    // 关闭电机
    drv8701e_pwm_1.set_duty(0);   
    drv8701e_pwm_2.set_duty(0);    
}


void pit_callback(void)
{
    printf("pit_callback!!!\n");
}



int main(int, char**) 
{
    drv8701e_pwm_1.get_dev_info(&drv8701e_pwm_1_info);
    drv8701e_pwm_2.get_dev_info(&drv8701e_pwm_2_info);

    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);


    // 创建一个定时器10ms周期，回调函数为pit_callback
    // pit_timer.init_ms(10, pit_callback);

    while(1)
    {
        if(duty >= 0)                                                           // 正转
        {
            drv8701e_dir_1.set_level(1);                                      // DIR输出高电平
            drv8701e_pwm_1.set_duty(duty * (MOTOR1_PWM_DUTY_MAX / 100));       // 计算占空比

            drv8701e_dir_2.set_level(1);                                      // DIR输出高电平
            drv8701e_pwm_2.set_duty(duty * (MOTOR2_PWM_DUTY_MAX / 100));       // 计算占空比
        }
        else
        {
            drv8701e_dir_1.set_level(0);                                      // DIR输出低电平
            drv8701e_pwm_1.set_duty(-duty * (MOTOR1_PWM_DUTY_MAX / 100));      // 计算占空比

            drv8701e_dir_2.set_level(0);                                      // DIR输出低电平
            drv8701e_pwm_2.set_duty(-duty * (MOTOR2_PWM_DUTY_MAX / 100));      // 计算占空比

        }

        if(dir)                                                         // 根据方向判断计数方向 本例程仅作参考
        {
            duty = MAX_DUTY;                                                    // 正向计数
            // if(duty >= MAX_DUTY)                                        // 达到最大值
            // dir = false;                                                // 变更计数方向
        }
        else
        {
            duty --;                                                    // 反向计数
            if(duty <= -MAX_DUTY)                                       // 达到最小值
            dir = true;                                                 // 变更计数方向
        }

        system_delay_ms(50);
    }
}

// **************************** 代码区域 ****************************

// *************************** 例程常见问题说明 ***************************
// 遇到问题时请按照以下问题检查列表检查
// 
// 问题1：终端提示未找到xxx文件
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//      使用本历程，就需要使用我们逐飞科技提供的内核，否则提示xxx文件找不到
//
// 问题2：电机不转或者模块输出电压无变化
//      如果使用主板测试，主板必须要用电池供电
//      检查模块是否正确连接供电 必须使用电源线供电 不能使用杜邦线
//      查看程序是否正常烧录，是否下载报错，确认正常按下复位按键
