#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
#include "sdram.h"
#include "lidar_data_process.h"
#include "lidar_n10.h"
#include "stm32_data_process.h"
#include "link_stm32.h"
#include "link_k230.h"
#include "usb_serial.h"
#include "app_buildmap.h"
#include "app_navigation.h"
#include "k230_data_process.h"
#include "app_vision.h"
#include "gpt.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdio.h>

volatile uint32_t g_mapping_cycles;
volatile uint32_t g_mapping_overruns;
volatile uint64_t g_mapping_last_us;
volatile uint64_t g_mapping_max_us;

/**
 * @brief 主入口：建图与规划 → 自动导航到终点 → 终点视觉跟随。
 *
 * @details 三段流程共用一条 20 ms 主循环，靠 g_navigation_state 切换控制权：
 *
 *          1) 建图与规划
 *             上位机经 USB 下发路线点，导航用 NavFn 在实时地图上规划路径
 *             （NAV_PLANNING → NAV_READY）。雷达点云经 LiDar_Data_Process()
 *             持续喂给建图模块（内部调 App_BuildMap_OnPoints），地图在车
 *             行驶过程中同步更新，导航避障和后面的视觉停车测距都用这张图。
 *
 *          2) 自动导航（NAV_RUNNING）
 *             按手柄 START 后从起点沿规划路径行驶，避障由导航的足迹检查
 *             App_NavigationPlan_CheckFootprint() 负责；到达最后一个路径点时
 *             导航自己把状态置为 NAV_DONE 并锁存制动。
 *
 *          3) 终点视觉（NAV_DONE → 视觉 SEARCH/TRACK → VISION_REACHED）
 *             视觉只在 NAV_DONE 下启动，随即向 K230 发开始命令：
 *             原地转找蓝色纸 → 用 offset_x 转向、box_h/area 定速走过去 →
 *             雷达沿车头方向测到墙的距离小于 VISION_CLEARANCE_MIN 时停车。
 */
void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;

    /* 片外 SDRAM 必须在第一次访问 0x68000000 之前初始化：
     * 栅格地图（1.08 MB）就放在那里，App_BuildMap_Init() 会立刻 memset 整块空间。
     * 自检不通就停在这里，避免后续对 0x68000000 的写入出错。 */
    if (!sdram_init()) {
        for (;;) {
            ;
        }
    }

    // 基础外设与 USB 通信通道
    UsbSerial_init();
    gpt_init();
    App_BuildMap_Init();

    // 雷达：导航建图与视觉停车测距共用同一份点云
    lidar_data_process_init();

    // 底盘通信（UART0）与 K230 视觉通信（UART4 + DMAC 通道 3）
    stm32_data_process_init();
    k230_data_process_init();

    // 应用层：导航从 NAV_IDLE 开始等上位机下发路线，视觉在 NAV_DONE 后接管
    App_Navigation_Init();
    App_Vision_Init();

    unsigned map_divider = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));
        uint64_t started_us = gpt_get_timestamp_us();

        taskENTER_CRITICAL();
        uint32_t stm32_snapshot = stm32_data_snapshot();
        uint32_t lidar_snapshot = lidar_data_snapshot();
        uint32_t k230_snapshot = k230_data_snapshot();
        taskEXIT_CRITICAL();

        // 底层解析：雷达解析内部会顺带更新建图，测距数据由此产生
        Stm32_Data_Process(stm32_snapshot);
        LiDar_Data_Process(lidar_snapshot);
        K230_Data_Process(k230_snapshot);

        // 视觉先跑、导航后跑，避免导航在切到视觉的周期多压制一次制动
        App_Vision_Poll();
        App_Navigation_Poll();

        if (++map_divider >= 10) {
            map_divider = 0;
            App_BuildMap_PublishSnapshot();
        }

        if (g_new_lidar_ready) {
            g_new_lidar_ready = false;
        }

        uint64_t elapsed_us = gpt_get_timestamp_us() - started_us;
        g_mapping_last_us = elapsed_us;
        if (elapsed_us > g_mapping_max_us) g_mapping_max_us = elapsed_us;
        if (elapsed_us > 20000U) g_mapping_overruns++;
        g_mapping_cycles++;
    }
}
