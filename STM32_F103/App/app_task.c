
#include "app_task.h"
#include "app_move.h"
#include "dev_ps2.h"

#define MOVE_TASK_PERIOD_MS      10U
#define PS2_CONFIG_INTERVAL_MS   1000U
void StartMoveTask(void *argument)
{
    (void)argument;

    App_Move_Init();
    Dev_Ps2_Init();
    App_Move_Stop();

    const uint32_t tick_hz = osKernelGetTickFreq();
    const uint32_t poll_ticks =
        (tick_hz * MOVE_TASK_PERIOD_MS + 999U) / 1000U;
    const uint32_t config_ticks =
        (tick_hz * PS2_CONFIG_INTERVAL_MS + 999U) / 1000U;

    const uint32_t task_delay = (poll_ticks > 0U) ? poll_ticks : 1U;
    const uint32_t retry_delay = (config_ticks > 0U) ? config_ticks : 1U;

    /* 等待接收器上电和配对 */
    osDelay((tick_hz / 5U) > 0U ? tick_hz / 5U : 1U);

    Dev_Ps2_ConfigureAnalog();
    uint32_t last_config_tick = osKernelGetTickCount();

    for (;;)
    {
        bool valid = Dev_Ps2_Read();

        if (!valid || !App_Move_ControlChassis(&ps2_data))
        {
            App_Move_Stop();

            /* 限制重新配置的频率 */
            uint32_t now = osKernelGetTickCount();

            if ((uint32_t)(now - last_config_tick) >= retry_delay)
            {
                Dev_Ps2_ConfigureAnalog();
                last_config_tick = osKernelGetTickCount();
            }

            osDelay(task_delay);
            continue;
        }

        osDelay(task_delay);
    }
}
