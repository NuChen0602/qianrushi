
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

#include <arpa/inet.h>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

// *************************** 例程硬件连接说明 ***************************
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  将2K30x核心板插到主板上面，确保插到底核心板与主板插座间没有缝隙即可
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
//  使用本历程，就需要使用我们逐飞科技提供的内核。
// 
//  目前仅支持两寸SPI屏幕
//  SPI 两寸屏 硬件引脚
//  SCL         查看 seekfree_2K30x_coreboard.dts 文件中 st7789v_or_st7735r 节点定义 默认 GPIO82
//  SDA         查看 seekfree_2K30x_coreboard.dts 文件中 st7789v_or_st7735r 节点定义 默认 GPIO84
//  RST         查看 seekfree_2K30x_coreboard.dts 文件中 st7789v_or_st7735r 节点定义 默认 GPIO81
//  DC          查看 seekfree_2K30x_coreboard.dts 文件中 st7789v_or_st7735r 节点定义 默认 GPIO71
//  CS          查看 seekfree_2K30x_coreboard.dts 文件中 st7789v_or_st7735r 节点定义 默认 GPIO85
//  BL          查看 seekfree_2K30x_coreboard.dts 文件中 st7789v_or_st7735r 节点定义 默认 GPIO75
//  GND         核心板电源地 GND
//  3V3         核心板 3V3 电源
// 
// *************************** 例程测试说明 ***************************
// 1.核心板烧录本例程 插在主板上 2寸IPS 显示模块插在主板的屏幕接口排座上 请注意引脚对应 不要插错
// 
// 2.电池供电 上电后 2寸IPS 屏幕亮起 显示数字等信息
// 
// 3.判断屏幕为SPI屏幕或者并口屏幕: 查看屏幕背面的PCB丝印，如果带有SPI字样就为SPI屏幕，否则为并口屏幕
//   例程文件夹下有不同类型2寸屏幕的区别示意图 "SPI和并口2寸屏幕区别.png"
//
// 如果发现现象与说明严重不符 请参照本文件最下方 例程常见问题说明 进行排查
//
// **************************** 代码区域 ****************************


zf_device_ips200 ips200;
zf_device_uvc uvc_dev;

static const uint16 SCREEN_WIDTH = 240;
static const uint16 HEADER_HEIGHT = 72;
static const uint16 MAP_Y = 80;
static const uint16 MAP_WIDTH = 240;
static const uint16 MAP_HEIGHT = 180;
static const uint16 MAP_STREAM_PORT = 2370;
static const uint32 MAP_FRAME_PIXELS = MAP_WIDTH * MAP_HEIGHT;
static const uint32 MAP_FRAME_BYTES = MAP_FRAME_PIXELS * sizeof(uint16);

static const uint16 COLOR_BACKGROUND = RGB565_BLACK;
static const uint16 COLOR_HEADER = RGB565_WHITE;
static const uint16 COLOR_DIVIDER = 0x5ACB;
static const uint16 COLOR_TEMP = 0xFD20;
static const uint16 COLOR_HUMIDITY = RGB565_CYAN;
static const uint16 COLOR_SMOKE = RGB565_RED;

static uint16 map_frame[MAP_FRAME_PIXELS];

enum ChineseGlyphIndex
{
    GLYPH_WEN,
    GLYPH_DU,
    GLYPH_SHI,
    GLYPH_YAN,
    GLYPH_WU,
};

// 16x16 Chinese glyphs, row-major, two bytes per row.
static const uint8 chinese_16x16[][32] =
{
    { 0x20, 0x00, 0x33, 0xFC, 0x1A, 0x0C, 0x02, 0x0C, 0x03, 0xFC, 0x62, 0x0C, 0x33, 0xFC, 0x00, 0x00,
      0x00, 0x00, 0x07, 0xFC, 0x1E, 0x94, 0x16, 0x94, 0x36, 0x94, 0x26, 0x94, 0x6F, 0xFF, 0x00, 0x00 }, // 温
    { 0x00, 0x80, 0x00, 0x80, 0x3F, 0xFE, 0x22, 0x10, 0x22, 0x18, 0x3F, 0xFE, 0x22, 0x18, 0x22, 0x18,
      0x23, 0xF0, 0x20, 0x00, 0x2F, 0xFC, 0x62, 0x18, 0x61, 0xB0, 0x40, 0xE0, 0x4F, 0xFC, 0x5C, 0x0E }, // 度
    { 0x00, 0x00, 0x33, 0xFC, 0x1A, 0x04, 0x02, 0x04, 0x03, 0xFC, 0x62, 0x04, 0x33, 0xFC, 0x00, 0x80,
      0x00, 0x90, 0x16, 0x96, 0x12, 0x96, 0x32, 0x94, 0x23, 0x9C, 0x60, 0x90, 0x6F, 0xFE, 0x00, 0x00 }, // 湿
    { 0x10, 0x00, 0x11, 0xFE, 0x11, 0x02, 0x13, 0x22, 0x57, 0x22, 0x55, 0x22, 0x5D, 0xFE, 0x51, 0x22,
      0x11, 0x32, 0x11, 0x32, 0x11, 0x72, 0x39, 0x4A, 0x3D, 0xCE, 0x25, 0x02, 0x61, 0xFE, 0x41, 0x02 }, // 烟
    { 0x01, 0x80, 0x7F, 0xFE, 0x41, 0x86, 0x5D, 0xFE, 0x5D, 0xF8, 0x05, 0x80, 0x0F, 0xF8, 0x3C, 0x30,
      0x07, 0xE0, 0x7D, 0x3E, 0x01, 0x00, 0x3F, 0xF8, 0x02, 0x08, 0x0C, 0x08, 0x38, 0x78, 0x00, 0x00 }, // 雾
};

static void fill_rect(uint16 x, uint16 y, uint16 width, uint16 height, uint16 color)
{
    for(uint16 row = 0; row < height; row++)
    {
        ips200.draw_line(x, y + row, x + width - 1, y + row, color);
    }
}

static void draw_chinese_glyph(uint16 x, uint16 y, uint8 glyph_index, uint16 color)
{
    const uint8 *glyph = chinese_16x16[glyph_index];

    for(uint8 row = 0; row < 16; row++)
    {
        uint16 bits = ((uint16)glyph[row * 2] << 8) | glyph[row * 2 + 1];
        for(uint8 col = 0; col < 16; col++)
        {
            if(bits & (0x8000 >> col))
            {
                ips200.draw_point(x + col, y + row, color);
            }
        }
    }
}

static void draw_sensor_label(uint16 x, uint16 y, uint8 first, uint8 second, uint16 color)
{
    draw_chinese_glyph(x, y, first, color);
    draw_chinese_glyph(x + 16, y, second, color);
}

static void draw_sensor_row(uint16 y, uint8 first, uint8 second, uint16 color, const char *value)
{
    draw_sensor_label(16, y, first, second, color);
    ips200.show_string(56, y, value);
}

static void draw_header(void)
{
    fill_rect(0, 0, SCREEN_WIDTH, HEADER_HEIGHT, COLOR_HEADER);
    ips200.draw_line(0, 23, SCREEN_WIDTH - 1, 23, COLOR_DIVIDER);
    ips200.draw_line(0, 47, SCREEN_WIDTH - 1, 47, COLOR_DIVIDER);
    ips200.draw_line(0, HEADER_HEIGHT - 1, SCREEN_WIDTH - 1, HEADER_HEIGHT - 1, COLOR_DIVIDER);

    draw_sensor_row(4, GLYPH_WEN, GLYPH_DU, COLOR_TEMP, ": 26.5C");
    draw_sensor_row(28, GLYPH_SHI, GLYPH_DU, COLOR_HUMIDITY, ": 55%");
    draw_sensor_row(52, GLYPH_YAN, GLYPH_WU, COLOR_SMOKE, ": 035ppm");
}

static bool read_exact(int fd, void *buffer, uint32 length)
{
    char *ptr = static_cast<char *>(buffer);
    uint32 received = 0;

    while(received < length)
    {
        int ret = recv(fd, ptr + received, length - received, 0);
        if(ret <= 0)
        {
            return false;
        }
        received += (uint32)ret;
    }
    return true;
}

static int create_map_stream_server(uint16 port)
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(server_fd < 0)
    {
        return -1;
    }

    int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr;
    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if(bind(server_fd, (sockaddr *)&addr, sizeof(addr)) < 0)
    {
        close(server_fd);
        return -1;
    }

    if(listen(server_fd, 1) < 0)
    {
        close(server_fd);
        return -1;
    }

    return server_fd;
}

static void draw_waiting_map(void)
{
    fill_rect(0, MAP_Y, SCREEN_WIDTH, MAP_HEIGHT, RGB565_GRAY);
    ips200.show_string(36, MAP_Y + 76, "WAIT MAP");
}

static int run_camera_mode(void)
{
    if(uvc_dev.init(UVC_PATH) < 0)
    {
        fill_rect(0, MAP_Y, SCREEN_WIDTH, MAP_HEIGHT, RGB565_GRAY);
        ips200.show_string(28, MAP_Y + 76, "NO CAMERA");
        while(1)
        {
            system_delay_ms(500);
        }
    }

    fill_rect(0, MAP_Y - 1, SCREEN_WIDTH, 1, COLOR_DIVIDER);
    fill_rect(0, MAP_Y, SCREEN_WIDTH, MAP_HEIGHT, COLOR_BACKGROUND);

    while(1)
    {
        if(uvc_dev.wait_image_refresh() == 0)
        {
            uint16 *rgb_image = uvc_dev.get_rgb_image_ptr();
            if(NULL != rgb_image)
            {
                ips200.show_rgb565_image(0, MAP_Y, rgb_image, UVC_WIDTH, UVC_HEIGHT, MAP_WIDTH, MAP_HEIGHT, 0);
            }
        }
        system_delay_ms(10);
    }
}

static int run_map_mode(void)
{
    int server_fd = create_map_stream_server(MAP_STREAM_PORT);
    if(server_fd < 0)
    {
        fill_rect(0, MAP_Y, SCREEN_WIDTH, MAP_HEIGHT, RGB565_GRAY);
        ips200.show_string(32, MAP_Y + 76, "TCP ERROR");
        while(1)
        {
            system_delay_ms(500);
        }
    }

    fill_rect(0, MAP_Y - 1, SCREEN_WIDTH, 1, COLOR_DIVIDER);

    while(1)
    {
        draw_waiting_map();
        int client_fd = accept(server_fd, NULL, NULL);
        if(client_fd < 0)
        {
            system_delay_ms(200);
            continue;
        }

        while(read_exact(client_fd, map_frame, MAP_FRAME_BYTES))
        {
            ips200.show_rgb565_image(0, MAP_Y, map_frame, MAP_WIDTH, MAP_HEIGHT, MAP_WIDTH, MAP_HEIGHT, 0);
        }
        close(client_fd);
    }
}

int main(int argc, char **argv)
{
    bool camera_mode = false;

    for(int arg = 1; arg < argc; arg++)
    {
        if(strcmp(argv[arg], "--camera") == 0)
        {
            camera_mode = true;
        }
    }

    ips200.init(FB_PATH);
    ips200.full(COLOR_BACKGROUND);
    draw_header();

    if(camera_mode)
    {
        return run_camera_mode();
    }
    return run_map_mode();
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
// 问题2：屏幕不显示
//      屏幕的初始化，是在开机的时候完成的，所以需要开启久久派之前插入屏幕
//      如果使用主板测试，主板必须要用电池供电 检查屏幕供电引脚电压
//      检查屏幕是不是插错位置了 检查引脚对应关系
//      如果对应引脚都正确 检查一下是否有引脚波形不对 需要有示波器
//      无法完成波形测试则复制一个GPIO例程将屏幕所有IO初始化为GPIO翻转电平 看看是否受控
