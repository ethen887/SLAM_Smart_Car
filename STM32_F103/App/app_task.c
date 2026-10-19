
#include "app_task.h"
#include "app_move.h"
#include "dev_ps2.h"
#include "dev_mg513.h"
#include "dev_lsm6dsr.h"
#include "app_imu_log.h"
#include "tim.h"
#include "FreeRTOS.h"
#include "task.h"

#define PS2_STARTUP_DELAY_MS     200U
#define PS2_POLL_PERIOD_MS       10U
#define PS2_CONFIG_INTERVAL_MS   1000U
#define PS2_TIMEOUT_MS           500U

// 最近一次成功接收PS2数据的时间
static uint32_t ps2_last_valid_tick = 0;
static bool ps2_has_valid_data = false;

// 实际控制任务间隔，供 Watch 检查；正常约为 0.010s。
volatile float move_task_dt_s;

/**
 * @brief 毫秒转换为FreeRTOS Tick，向上取整
 * @param ms 时间，单位ms
 * @param tick_hz RTOS Tick频率
 * @return 对应的Tick数量，最小为1
 */
static uint32_t App_MsToTicks(uint32_t ms, uint32_t tick_hz)
{
    uint32_t ticks = (uint32_t)(((uint64_t)ms * tick_hz + 999U) / 1000U);
    return ticks > 0U ? ticks : 1U;
}

/**
 * @brief 电机运动控制任务
 * @note TIM1每10ms通知一次，负责编码器反馈和底盘控制
 */
void StartMoveTask(void *argument)
{
    (void)argument;

    App_Move_Init();
    App_Move_Stop();

    // 外设已由 main 初始化；IMU 失败不阻止原有电机任务运行。
    Dev_Lsm6dsr_Init();
    App_ImuLog_Init();

    const uint32_t tick_hz = osKernelGetTickFreq();
    if (tick_hz == 0U) return;

    const uint32_t timeout_ticks = App_MsToTicks(PS2_TIMEOUT_MS, tick_hz);

    // 保存任务句柄，用于TIM1中断通知
    move_task_handle = osThreadGetId();

    // 开启TIM1更新中断
    __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    uint32_t last_tick = osKernelGetTickCount();

    for (;;)
    {
        // 等待TIM1周期通知
        uint32_t flags = osThreadFlagsWait(
            MOVE_TASK_UPDATE_FLAG,
            osFlagsWaitAny,
            osWaitForever
        );

        if (flags & osFlagsError) continue;

        // 计算两次采样的实际时间间隔
        uint32_t now = osKernelGetTickCount();
        uint32_t elapsed = now - last_tick;
        if (elapsed == 0U) continue;

        last_tick = now;
        float dt = (float)elapsed / (float)tick_hz;
        move_task_dt_s = dt;

        // 更新左右电机角度、速度
        Dev_MG513_Update(left_motor, dt);
        Dev_MG513_Update(right_motor, dt);

        // 获取PS2通信状态及最近一次成功接收时间
        bool valid;
        uint32_t last_valid_tick;

        taskENTER_CRITICAL();
        valid = ps2_has_valid_data;
        last_valid_tick = ps2_last_valid_tick;
        taskEXIT_CRITICAL();

        // 未收到有效数据或通信超时，停止电机
        if (!valid || (uint32_t)(now - last_valid_tick) >= timeout_ticks)
        {
            App_Move_Stop();
        }
        // 使用最新的PS2数据执行底盘控制
        else if (!App_Move_ControlChassis(&ps2_data))
        {
            App_Move_Stop();
        }

        // 每 10ms 有上限地取出 FIFO 积压样本；未连接手柄也采集。
        // IMU 保持 104Hz，样本间隔由设备接口提供，不使用电机控制 dt。
        Dev_Lsm6dsr_Update();
        App_ImuLog_UpdateStatus(dt);
    }
}

/**
 * @brief PS2手柄通信任务
 * @note 独立读取手柄，通信失败时保留上一次有效数据
 */
void StartRemoteTask(void *argument)
{
    (void)argument;

    Dev_Ps2_Init();

    const uint32_t tick_hz = osKernelGetTickFreq();
    if (tick_hz == 0U) return;

    const uint32_t startup_ticks =
        App_MsToTicks(PS2_STARTUP_DELAY_MS, tick_hz);

    const uint32_t poll_ticks =
        App_MsToTicks(PS2_POLL_PERIOD_MS, tick_hz);

    const uint32_t retry_ticks =
        App_MsToTicks(PS2_CONFIG_INTERVAL_MS, tick_hz);

    // 等待PS2接收器上电并配置模拟模式
    osDelay(startup_ticks);
    Dev_Ps2_ConfigureAnalog();

    uint32_t last_config_tick = osKernelGetTickCount();

    for (;;)
    {
        bool valid = Dev_Ps2_Read();

        if (valid)
        {
            // 读取成功，记录最近一次有效通信时间
            taskENTER_CRITICAL();
            ps2_last_valid_tick = osKernelGetTickCount();
            ps2_has_valid_data = true;
            taskEXIT_CRITICAL();
        }
        else
        {
            // 读取失败，保留上一次有效的ps2_data
            uint32_t now = osKernelGetTickCount();

            // 每隔1秒尝试重新配置手柄
            if ((uint32_t)(now - last_config_tick) >= retry_ticks)
            {
                Dev_Ps2_ConfigureAnalog();
                last_config_tick = osKernelGetTickCount();
            }
        }

        // 等待下一次PS2读取
        osDelay(poll_ticks);
    }
}
