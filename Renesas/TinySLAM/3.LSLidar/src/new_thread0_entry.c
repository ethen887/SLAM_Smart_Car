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

/**
 * @brief 主入口：建图 → 自主导航 → 终点视觉，三段靠 g_navigation_state 切换。
 *
 * @details
 *          第一段 建图（USB 接着，手柄控车）
 *            电脑用 USB 连 RA8P1，手柄直接控制底盘沿场地走一遍。
 *            底盘里程计经 UART0 送来位姿，雷达点云经 LiDar_Data_Process()
 *            喂给建图模块（内部调 App_BuildMap_OnPoints），地图实时长出来并
 *            经 USB 回传给电脑，由电脑确定起点和终点坐标。
 *            这一段没有开始/结束开关：只要车在动、雷达在转，建图就在进行。
 *
 *          第二段 自主导航（可以拔掉 USB）
 *            电脑把终点坐标经 USB 下发，导航在已建好的地图上规划出路径，
 *            回放预览确认后进入 NAV_READY。此时拔掉 USB 不影响后续运行：
 *            导航只在收到指令时才读 USB，没有“电脑心跳”这类依赖，
 *            状态回传发不出去也只会被 UsbSerial 丢掉，不会阻塞主循环。
 *            手柄按 START 后进入 NAV_RUNNING，沿路径行驶到终点并置 NAV_DONE。
 *
 *          第三段 终点视觉
 *            NAV_DONE 后视觉自动接管，向 K230 发开始命令并原地转找蓝色 A4 纸，
 *            找到后用 offset_x 转向、box_h/area 定速度靠近，
 *            雷达沿车头方向测到墙的距离小于 VISION_CLEARANCE_MIN 时停车。
 *
 * @note 地图放在片外 SDRAM，掉电即失，所以建图和导航必须在同一次上电内完成。
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

    // 应用层：导航从 NAV_IDLE 开始等电脑下发路线，视觉在 NAV_DONE 后接管
    App_Navigation_Init();
    App_Vision_Init();

    unsigned map_divider = 0;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(20));

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

        // 地图快照只用于电脑端显示，拔掉 USB 后这次调用会直接返回失败，不阻塞
        if (++map_divider >= 10) {
            map_divider = 0;
            App_BuildMap_PublishSnapshot();
        }

        if (g_new_lidar_ready) {
            g_new_lidar_ready = false;
        }
    }
}
