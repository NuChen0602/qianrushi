
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
//  主板按键       单片机管脚
//  KEY_0         查看 seekfree_2K30x_coreboard.dts 文件中 zf_gpio_key_0       节点定义 默认 GPIO77
//  KEY_1         查看 seekfree_2K30x_coreboard.dts 文件中 zf_gpio_key_1       节点定义 默认 GPIO78
//  KEY_2         查看 seekfree_2K30x_coreboard.dts 文件中 zf_gpio_key_2       节点定义 默认 GPIO79
//  KEY_3         查看 seekfree_2K30x_coreboard.dts 文件中 zf_gpio_key_3       节点定义 默认 GPIO80
//  SWITCH_1      查看 seekfree_2K30x_coreboard.dts 文件中 zf_gpio_switch_1    节点定义 默认 GPIO68
//  SWITCH_2      查看 seekfree_2K30x_coreboard.dts 文件中 zf_gpio_switch_2    节点定义 默认 GPIO69

// *************************** 例程测试说明 ***************************
// 1.将2K30x核心板插到主板上面 主板使用电池供电 下载本例程
// 
// 2.打开终端可以看到:
// key_0 = x.
// key_1 = x.
// key_2 = x.
// key_3 = x.
// switch_1 = x.
// switch_2 = x.
// 3.按下按键即可看到不同的输出状态
//
// **************************** 代码区域 ****************************

#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4
#define SWITCH_1_PATH     ZF_GPIO_SWITCH_1
#define SWITCH_2_PATH     ZF_GPIO_SWITCH_2

zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);
zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);
zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);
zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);
zf_driver_gpio  switch_1(SWITCH_1_PATH, O_RDWR);
zf_driver_gpio  switch_2(SWITCH_2_PATH, O_RDWR);

int main(int, char**) 
{

    while(1)
    {
        printf("key_1 = %d\r\n",    key_1.get_level());
        printf("key_2 = %d\r\n",    key_2.get_level());
        printf("key_3 = %d\r\n",    key_3.get_level());
        printf("key_4 = %d\r\n",    key_4.get_level());
        printf("switch_1 = %d\r\n", switch_1.get_level());
        printf("switch_2 = %d\r\n", switch_2.get_level());

        system_delay_ms(100);
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