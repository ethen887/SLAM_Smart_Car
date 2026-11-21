#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

/* 车体坐标系：X 向前、Y 向左、Z 向上。角度单位 rad，角速度 rad/s，加速度 m/s²。 */
typedef struct {
    bool initialized;          /* 四元数已建立，姿态角可用。 */
    bool calibrated;           /* 陀螺零偏标定完成。 */
    bool still;                /* 本轮判定为静止，航向已冻结。 */
    float roll, pitch, yaw;    /* 姿态角；yaw 为相对标定时刻的航向。 */
    float gyro_bias[3];        /* 陀螺零偏估计，rad/s。 */
    float accel_forward;       /* 机体系前向运动加速度，已去重力，m/s²。 */
    uint32_t stream_id;        /* 数据流编号，中断后递增。 */
    uint32_t calibration_samples; /* 当前静止标定累计样本数。 */
} AttitudeData_t;

extern volatile AttitudeData_t attitude_data;

/** @brief 清空姿态、零偏和航向积分状态。 */
void App_Attitude_Init(void);

/** @brief 由运动任务每周期更新；静止标志只用于零偏标定和航向冻结。 */
void App_Attitude_SetStationary(bool stationary);

/* 输入为传感器坐标系样本，单位 m/s² 或 rad/s，按 FIFO 顺序调用。 */
void App_Attitude_Process(bool gyro, const float value[3], float dt, uint32_t stream_id);

/**
 * @brief 取走自上次调用以来累计的航向增量，返回数据是否可用于融合。
 * @param task_dt 控制周期，s；用于判断累计时间是否合理。
 * @param continuous false 表示 FIFO 发生错误或积压，本段增量不可信。
 * @param delta 输出航向增量，rad；仅在返回 true 时有效。
 * @note 增量为真实积分结果，不按控制周期缩放；调用方直接叠加到航向即可。
 */
bool App_Attitude_TakeYawDelta(float task_dt, bool continuous, float *delta);

#endif
