#ifndef APP_ATTITUDE_H
#define APP_ATTITUDE_H

#include <stdbool.h>
#include <stdint.h>

/* Body axes: X forward, Y left, Z up. Values are radians and rad/s. */
typedef struct {
    bool initialized, calibrated;
    float roll, pitch, yaw;
    float gyro_bias[3];
    uint32_t stream_id, calibration_samples;
} AttitudeData_t;

extern volatile AttitudeData_t attitude_data;

void App_Attitude_Init(void);
void App_Attitude_SetStationary(bool stationary);
/* Inputs are sensor-frame samples, in m/s^2 or rad/s, in FIFO order. */
void App_Attitude_Process(bool gyro, const float value[3], float dt, uint32_t stream_id);
/* Consume each interval once. continuous=false discards it after a FIFO fault/backlog. */
bool App_Attitude_TakeYawDelta(float task_dt, bool continuous, float *delta);

#endif
