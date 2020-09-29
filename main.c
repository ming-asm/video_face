#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "fpioa.h"
#include "lcd.h"
#include "board_config.h"
#include "plic.h"
#include "sysctl.h"
#include "uarths.h"
#include "nt35310.h"
#include "utils.h"
#include "kpu.h"
#include "region_layer.h"
#include "w25qxx.h"
#include "incbin.h"
#include <math.h>
#include "bsp.h"
#include "gpiohs.h"
#include "dvp.h"
#include "ov5640.h"
#include "image_process.h"
#include "iomem.h"
#define INCBIN_STYLE INCBIN_STYLE_SNAKE
#define INCBIN_PREFIX


#define PLL0_OUTPUT_FREQ 800000000UL
#define PLL1_OUTPUT_FREQ 400000000UL

#define CLASS_NUMBER 1

#define LOAD_KMODEL_FROM_FLASH 0

// extern const unsigned char gImage_image[] __attribute__((aligned(128)));

volatile uint16_t g_dvp_finish_flag;

static image_t kpu_image, display_image;
static obj_info_t detect_info;

#if LOAD_KMODEL_FROM_FLASH
#define KMODEL_SIZE (1351976)
uint8_t *model_data;
#else
INCBIN(model, "mobile_yolo_0908214537.kmodel");
#endif
kpu_model_context_t task;
// kpu_model_context_t face_detect_task;
static region_layer_t detect_rl;
// static region_layer_t face_detect_rl;
// volatile uint8_t g_ai_done_flag;
volatile uint32_t g_ai_done_flag;
// static uint16_t lcd_gram[320 * 240] __attribute__((aligned(32)));

static int ai_done(void *ctx)
{
    g_ai_done_flag = 1;
    return 0;
}

#define ANCHOR_NUM 3

static float g_anchor[ANCHOR_NUM * 2] = {
   0.32484866, 0.59616056, 0.1797674, 0.33981681, 0.08770987, 0.16554958};

#if BOARD_LICHEEDAN
static void io_mux_init(void)
{
    /* Init DVP IO map and function settings */
    fpioa_set_function(42, FUNC_CMOS_RST);
    fpioa_set_function(44, FUNC_CMOS_PWDN);
    fpioa_set_function(46, FUNC_CMOS_XCLK);
    fpioa_set_function(43, FUNC_CMOS_VSYNC);
    fpioa_set_function(45, FUNC_CMOS_HREF);
    fpioa_set_function(47, FUNC_CMOS_PCLK);
    fpioa_set_function(41, FUNC_SCCB_SCLK);
    fpioa_set_function(40, FUNC_SCCB_SDA);

    /* Init SPI IO map and function settings */
    fpioa_set_function(38, FUNC_GPIOHS0 + DCX_GPIONUM);
    fpioa_set_function(36, FUNC_SPI0_SS3);
    fpioa_set_function(39, FUNC_SPI0_SCLK);
    fpioa_set_function(37, FUNC_GPIOHS0 + RST_GPIONUM);

    sysctl_set_spi0_dvp_data(1);
}

static void io_set_power(void)
{
    /* Set dvp and spi pin to 1.8V */
    sysctl_set_power_mode(SYSCTL_POWER_BANK6, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK7, SYSCTL_POWER_V18);
}

#else
static void io_mux_init(void)
{
    /* Init DVP IO map and function settings */
    fpioa_set_function(11, FUNC_CMOS_RST);
    fpioa_set_function(13, FUNC_CMOS_PWDN);
    fpioa_set_function(14, FUNC_CMOS_XCLK);
    fpioa_set_function(12, FUNC_CMOS_VSYNC);
    fpioa_set_function(17, FUNC_CMOS_HREF);
    fpioa_set_function(15, FUNC_CMOS_PCLK);
    fpioa_set_function(10, FUNC_SCCB_SCLK);
    fpioa_set_function(9, FUNC_SCCB_SDA);

    /* Init SPI IO map and function settings */
    fpioa_set_function(8, FUNC_GPIOHS0 + 2);
    fpioa_set_function(6, FUNC_SPI0_SS3);
    fpioa_set_function(7, FUNC_SPI0_SCLK);

    sysctl_set_spi0_dvp_data(1);
    fpioa_set_function(26, FUNC_GPIOHS0 + 8);
    gpiohs_set_drive_mode(8, GPIO_DM_INPUT);
}

static void io_set_power(void)
{
    /* Set dvp and spi pin to 1.8V */
    sysctl_set_power_mode(SYSCTL_POWER_BANK0, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK1, SYSCTL_POWER_V18);
    sysctl_set_power_mode(SYSCTL_POWER_BANK2, SYSCTL_POWER_V18);
}
#endif

// DVP中断函数
static int dvp_irq(void *ctx)
{
    if (dvp_get_interrupt(DVP_STS_FRAME_FINISH))
    {
        dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);
        dvp_clear_interrupt(DVP_STS_FRAME_FINISH);
        g_dvp_finish_flag = 1;
    }
    else
    {
        dvp_start_convert();
        dvp_clear_interrupt(DVP_STS_FRAME_START);
    }
    return 0;
}

#if (CLASS_NUMBER > 1)
typedef struct
{
    char *str;
    uint16_t color;
    uint16_t height;
    uint16_t width;
    uint32_t *ptr;
} class_lable_t;

class_lable_t class_lable[CLASS_NUMBER] =
{
    {"aeroplane", GREEN},
    {"bicycle", GREEN},
    {"bird", GREEN},
    {"boat", GREEN},
    {"bottle", 0xF81F},
    {"bus", GREEN},
    {"car", GREEN},
    {"cat", GREEN},
    {"chair", 0xFD20},
    {"cow", GREEN},
    {"diningtable", GREEN},
    {"dog", GREEN},
    {"horse", GREEN},
    {"motorbike", GREEN},
    {"person", 0xF800},
    {"pottedplant", GREEN},
    {"sheep", GREEN},
    {"sofa", GREEN},
    {"train", GREEN},
    {"tvmonitor", 0xF9B6}
};

static uint32_t lable_string_draw_ram[115 * 16 * 8 / 2];
#endif

static void lable_init(void)
{
#if (CLASS_NUMBER > 1)
    uint8_t index;

    class_lable[0].height = 16;
    class_lable[0].width = 8 * strlen(class_lable[0].str);
    class_lable[0].ptr = lable_string_draw_ram;
    lcd_ram_draw_string(class_lable[0].str, class_lable[0].ptr, BLACK, class_lable[0].color);
    for (index = 1; index < CLASS_NUMBER; index++) {
        class_lable[index].height = 16;
        class_lable[index].width = 8 * strlen(class_lable[index].str);
        class_lable[index].ptr = class_lable[index - 1].ptr + class_lable[index - 1].height * class_lable[index - 1].width / 2;
        lcd_ram_draw_string(class_lable[index].str, class_lable[index].ptr, BLACK, class_lable[index].color);
    }
#endif
}

// static void drawboxes(uint32_t x1, uint32_t y1, uint32_t x2, uint32_t y2, uint32_t class, float prob)
// {
//     if (x1 >= 320)
//         x1 = 319;
//     if (x2 >= 320)
//         x2 = 319;
//     if (y1 >= 223)
//         y1 = 222;
//     if (y2 >= 223)
//         y2 = 222;
// 
// #if (CLASS_NUMBER > 1)
//     lcd_draw_rectangle(x1, y1, x2, y2, 2, class_lable[class].color);
//     lcd_draw_picture(x1 + 1, y1 + 1, class_lable[class].width, class_lable[class].height, class_lable[class].ptr);
// #else
//     lcd_draw_rectangle(x1, y1, x2, y2, 2, RED);
// #endif
// }

void rgb888_to_lcd(uint8_t *src, uint16_t *dest, size_t width, size_t height)
{
    size_t i, chn_size = width * height;
    for (size_t i = 0; i < width * height; i++)
    {
        uint8_t r = src[i];
        uint8_t g = src[chn_size + i];
        uint8_t b = src[chn_size * 2 + i];

        uint16_t rgb = ((r & 0b11111000) << 8) | ((g & 0b11111100) << 3) | (b >> 3);
        size_t d_i = i % 2 ? (i - 1) : (i + 1);
        dest[d_i] = rgb;
    }
}

static void draw_edge(uint32_t *gram, obj_info_t *obj_info, uint32_t index, uint16_t color)
{
    uint32_t data = ((uint32_t)color << 16) | (uint32_t)color;
    uint32_t *addr1, *addr2, *addr3, *addr4, x1, y1, x2, y2;

    x1 = obj_info->obj[index].x1;
    y1 = obj_info->obj[index].y1;
    x2 = obj_info->obj[index].x2;
    y2 = obj_info->obj[index].y2;
    printf("%d, %d, %d, %d \n", x1, y1, x2, y2);
    if (x1 <= 0)
        x1 = 1;
    if (x2 >= 319)
        x2 = 318;
    if (y1 <= 0)
        y1 = 1;
    if (y2 >= 223)
        y2 = 222;

    addr1 = gram + (320 * y1 + x1) / 2;
    addr2 = gram + (320 * y1 + x2 - 8) / 2;
    addr3 = gram + (320 * (y2 - 1) + x1) / 2;
    addr4 = gram + (320 * (y2 - 1) + x2 - 8) / 2;
    for (uint32_t i = 0; i < 4; i++)
    {
        *addr1 = data;
        *(addr1 + 160) = data;
        *addr2 = data;
        *(addr2 + 160) = data;
        *addr3 = data;
        *(addr3 + 160) = data;
        *addr4 = data;
        *(addr4 + 160) = data;
        addr1++;
        addr2++;
        addr3++;
        addr4++;
    }
    addr1 = gram + (320 * y1 + x1) / 2;
    addr2 = gram + (320 * y1 + x2 - 2) / 2;
    addr3 = gram + (320 * (y2 - 8) + x1) / 2;
    addr4 = gram + (320 * (y2 - 8) + x2 - 2) / 2;
    for (uint32_t i = 0; i < 8; i++)
    {
        *addr1 = data;
        *addr2 = data;
        *addr3 = data;
        *addr4 = data;
        addr1 += 160;
        addr2 += 160;
        addr3 += 160;
        addr4 += 160;
    }
}

int main(void)
{
    /* Set CPU and dvp clk */
    sysctl_pll_set_freq(SYSCTL_PLL0, PLL0_OUTPUT_FREQ);
    sysctl_pll_set_freq(SYSCTL_PLL1, PLL1_OUTPUT_FREQ);
    sysctl_clock_enable(SYSCTL_CLOCK_AI);
    uarths_init();

    io_set_power();
    io_mux_init();  
    plic_init();

    /* flash init */
    printf("flash init\n");
    w25qxx_init(3, 0);
    w25qxx_enable_quad_mode();


#if LOAD_KMODEL_FROM_FLASH
    printf("flash init\n");
    w25qxx_init(3, 0);
    w25qxx_enable_quad_mode();
    model_data = (uint8_t *)malloc(KMODEL_SIZE);
    w25qxx_read_data(0xA00000, model_data, KMODEL_SIZE, W25QXX_QUAD_FAST);
// #else
//     uint8_t *model_data_align = model_data;
#endif
    lable_init();

    /* LCD init */
    printf("LCD init\n");
    lcd_init();

#if BOARD_LICHEEDAN
    lcd_set_direction(DIR_YX_RLDU);
#else
    lcd_set_direction(DIR_YX_RLUD);
#endif

    lcd_clear(BLACK);
    /* DVP init */
    printf("DVP init\n");
    #if OV5640
    dvp_init(16);
    dvp_set_xclk_rate(12000000);
    dvp_enable_burst();
    dvp_set_output_enable(0, 1);
    dvp_set_output_enable(1, 1);
    dvp_set_image_format(DVP_CFG_RGB_FORMAT);
    dvp_set_image_size(320, 224);
    ov5640_init();
    #else
    dvp_init(8);
    dvp_set_xclk_rate(24000000);
    dvp_enable_burst();
    dvp_set_output_enable(0, 1);
    dvp_set_output_enable(1, 1);
    dvp_set_image_format(DVP_CFG_RGB_FORMAT);
    dvp_set_image_size(320, 240);
    ov2640_init();
    #endif

    kpu_image.pixel = 3;
    kpu_image.width = 320;
    kpu_image.height = 224;
    image_init(&kpu_image);

    display_image.pixel = 2;
    display_image.width = 320;
    display_image.height = 224;
    image_init(&display_image);
    printf("===1===");
    dvp_set_ai_addr((uint32_t)kpu_image.addr, (uint32_t)(kpu_image.addr + 320 * 224), 
                    (uint32_t)(kpu_image.addr + 320 * 224 * 2));//设置 AI 存放图像的地址，供 AI 模块进行算法处理。
    printf("===2===");
    dvp_set_display_addr((uint32_t)display_image.addr);//设置采集图像在内存中的存放地址，可以用来显示
    printf("===3===");
    dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 0);//配置 DVP 中断类型。
    printf("===4===");
    dvp_disable_auto();//禁用自动接收图像模式。
    /* DVP interrupt config */
    printf("DVP interrupt config\n");
 
    plic_set_priority(IRQN_DVP_INTERRUPT, 1);//设置中断优先级
 
    plic_irq_register(IRQN_DVP_INTERRUPT, dvp_irq, NULL);//注册外部中断函数。
    
    plic_irq_enable(IRQN_DVP_INTERRUPT);//使能外部中断。

    /* enable global interrupt */
    sysctl_enable_irq();//使能系统中断，如果使用中断一定要开启系统中断

    /* system start */
    printf("system start\n");
    /* init kpu */
    if (kpu_load_kmodel(&task, model_data) != 0)
    {
        printf("\nmodel init error\n");
        while (1);
    }

    detect_rl.anchor_number = ANCHOR_NUM;
    detect_rl.anchor = g_anchor;
    detect_rl.threshold = 0.5;
    detect_rl.nms_value = 0.3;
    // region_layer_init(&detect_rl, 10, 7, 125, 320, 240);
    region_layer_init(&detect_rl, 10, 7, 18, kpu_image.width, kpu_image.height);
    uint64_t time_last = sysctl_get_time_us();
    uint64_t time_now = sysctl_get_time_us();
    uint64_t time1 = sysctl_get_time_us();
    uint64_t time2 = sysctl_get_time_us();
    int time_count = 0;
    while (1)
    {
        g_dvp_finish_flag = 0;
        dvp_clear_interrupt(DVP_STS_FRAME_START | DVP_STS_FRAME_FINISH);//清除中断
        dvp_config_interrupt(DVP_CFG_START_INT_ENABLE | DVP_CFG_FINISH_INT_ENABLE, 1);//配置 DVP 中断类型。设置图像开始和结束中断状态，使能
        while (g_dvp_finish_flag == 0)
            ;
        /* run face detect */
        g_ai_done_flag = 0;
        time1 = sysctl_get_time_us();  
        //task:KPU 任务句柄, kpu_image.addr:输入图像数据, DMAC_CHANNEL5:DMA 通道, ai_done:运算输出结果
        kpu_run_kmodel(&task, kpu_image.addr, DMAC_CHANNEL5, ai_done, NULL);
        while(!g_ai_done_flag);
        float *output;
        size_t output_size;
        kpu_get_output(&task, 0, (uint8_t **)&output, &output_size);
                     
        detect_rl.input = output;
        // printf("%f\n", *detect_rl.input);   // 0.000000  
        // time1 = sysctl_get_time_us();   
        region_layer_run(&detect_rl, &detect_info);
        time2 = sysctl_get_time_us();
        printf("TIME:%fms\n", (time2 - time1)/1000.0);
        /* run key point detect */
        for (uint32_t obj_cnt = 0; obj_cnt < detect_info.obj_number; obj_cnt++)
        {
            draw_edge((uint32_t *)display_image.addr, &detect_info, obj_cnt, GREEN);
        }
        /* display result */
        lcd_draw_picture(0, 0, 320, 224, (uint32_t *)display_image.addr);
        // region_layer_draw_boxes(&detect_rl, &detect_info);
        time_count ++;
        if(time_count % 100 == 0)
        {
            time_now = sysctl_get_time_us();
            printf("SPF:%fms\n", (time_now - time_last)/1000.0/100);
            printf("TIME:%fms\n", (time_now - time1)/1000.0/100);
            time_last = time_now;
        }
    }
}

