
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

// *************************** 例程测试说明 ***************************
// 1. 主板使用电池供电，下载本例程运行
// 2. 打开终端可看到完整的运行日志及正确打印数据：
//    包含文件初始化、数据写入、指针复位、数据读取、结果校验全流程信息
//    最终输出：read_u32_dat = 1234567  |  read_float_dat = 1.234560
// 3. 基于Linux一切皆文件的理念，演示文件存储参数的完整读写方法
// ********************************************************************

// 定义文件路径与名称
#define FILENAME_U32     "parameter_u32.txt"    // 存储整型数据的文件
#define FILENAME_FLOAT   "parameter_float.txt"  // 存储浮点型数据的文件

// 创建文件操作对象，w+模式：文件不存在自动创建 | 文件存在则清空内容 | 支持可读可写
zf_driver_file_string file_u32(FILENAME_U32,    "w+");
zf_driver_file_string file_float(FILENAME_FLOAT,"w+");

// 定义读写数据变量
uint32  write_u32_dat   = 1234567;     // 待写入的整型数据
float   write_float_dat = 3.23456;     // 待写入的浮点型数据
uint32  read_u32_dat    = 0;           // 存储读取的整型数据
float   read_float_dat  = 0;           // 存储读取的浮点型数据

int main(int, char**) 
{
    // 定义字符串缓冲区，用于格式化数据和存储读取内容
    char buffer[100] = {0};
    
    // ===================== 程序运行开始 打印头信息 =====================
    printf("=====================================\r\n");
    printf("     文件读写测试程序 开始运行\r\n");
    printf("=====================================\r\n");
    printf("待写入整型数据  : %d\r\n", write_u32_dat);
    printf("待写入浮点数据  : %.6f\r\n", write_float_dat);
    printf("=====================================\r\n\r\n");

    /************************* 【第一步】整型数据 完整读写操作 *************************/
    printf("【整型数据操作】---------------------\r\n");
    // 1. 格式化整型数据为字符串
    sprintf(buffer, "%d", write_u32_dat);
    printf("1. 整型数据格式化完成，内容: %s\r\n", buffer);
    
    // 2. 写入文件
    if(0 == file_u32.write_string(buffer))
    {
        printf("2. 整型数据写入文件【%s】成功！\r\n", FILENAME_U32);
    }
    else
    {
        printf("2. 整型数据写入文件【%s】失败！\r\n", FILENAME_U32);
    }

    // 3. 复位文件指针到开头（写完必须复位才能读）
    file_u32.rewind_file();
    printf("3. 文件指针已复位到文件开头\r\n");

    // 4. 清空缓冲区+读取数据
    memset(buffer, 0, sizeof(buffer));
    if(0 == file_u32.read_string(buffer))
    {
        printf("4. 从文件读取字符串成功，内容: %s\r\n", buffer);
    }
    else
    {
        printf("4. 从文件读取字符串失败！\r\n");
    }

    // 5. 字符串转整型+打印结果
    read_u32_dat = atoi(buffer);
    printf("5. 字符串转整型完成，最终读取结果: ");
    printf("read_u32_dat = %d\r\n\r\n", read_u32_dat);

    /************************* 【第二步】浮点型数据 完整读写操作 *************************/
    printf("【浮点数据操作】---------------------\r\n");
    // 1. 格式化浮点数据为字符串
    sprintf(buffer, "%f", write_float_dat);
    printf("1. 浮点数据格式化完成，内容: %s\r\n", buffer);
    
    // 2. 写入文件
    if(0 == file_float.write_string(buffer))
    {
        printf("2. 浮点数据写入文件【%s】成功！\r\n", FILENAME_FLOAT);
    }
    else
    {
        printf("2. 浮点数据写入文件【%s】失败！\r\n", FILENAME_FLOAT);
    }

    // 3. 复位文件指针到开头（写完必须复位才能读）
    file_float.rewind_file();
    printf("3. 文件指针已复位到文件开头\r\n");

    // 4. 清空缓冲区+读取数据
    memset(buffer, 0, sizeof(buffer));
    if(0 == file_float.read_string(buffer))
    {
        printf("4. 从文件读取字符串成功，内容: %s\r\n", buffer);
    }
    else
    {
        printf("4. 从文件读取字符串失败！\r\n");
    }

    // 5. 字符串转浮点+打印结果
    read_float_dat = atof(buffer);
    printf("5. 字符串转浮点完成，最终读取结果: ");
    printf("read_float_dat = %f\r\n\r\n", read_float_dat);

    // ===================== 程序运行结束 打印尾信息 =====================
    printf("=====================================\r\n");
    printf("     文件读写测试程序 运行结束\r\n");
    printf("=====================================\r\n");
    
    return 0;
}