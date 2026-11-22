#include "hal_data.h"
#include "new_thread0.h"
#include "uart.h"
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
 * @brief 主入口：建图（先跑一遍）→ 自动导航到终点 → 终点视觉跟随。
 *
 * @details 三段流程共用一条 20 ms 主循环，靠 g_navigation_state 切换控制权：
 *
 *          1) 建图（NAV_IDLE → NAV_RECORDING → NAV_READY）
 *             上电后导航处于 NAV_IDLE。上位机经 USB 下发“开始记录”命令后进入
 *             NAV_RECORDING，人推着车（或遥控）把场地走一遍，雷达点云经
 *             LiDar_Data_Process() 持续喂给建图模块（内部调 App_BuildMap_OnPoints），
 *             地图在这一遍里建出来。走到起点按“停止记录”进入 NAV_READY。
 *
 *          2) 自动导航（NAV_RUNNING）
 *             上位机下发“开始导航”后车从起点自动跑到终点，
 *             避障由导航的 footprint_error() 负责。到达最后一个目标点时
 *             导航自己把状态置为 NAV_DONE 并锁存制动。
 *
 *          3) 终点视觉（NAV_DONE → 视觉 SEARCH/TRACK → VISION_REACHED）
 *             视觉只在 NAV_DONE 下启动，随即向 K230 发开始命令：
 *             原地转找蓝色纸 → 用 offset_x 转向、box_h/area 定速走过去 →
 *             雷达沿车头方向测到墙的距离小于 VISION_CLEARANCE_MIN 时停车。
 *             停车测距复用的正是第 1 段建好的那张图，所以能读到终点墙面。
 */
void new_thread0_entry(void *pvParameters)
{
    (void)pvParameters;

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
