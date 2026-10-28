#include "app_attitude.h"
#include <math.h>

/* Change this signed permutation to match the actual sensor mounting. */
#ifndef IMU_BODY_X
#define IMU_BODY_X  1
#define IMU_BODY_Y  2
#define IMU_BODY_Z  3
#endif
#define GRAVITY 9.80665f
#define CAL_SAMPLES 200U
#define ABS_AXIS(x) ((x) < 0 ? -(x) : (x))
_Static_assert(ABS_AXIS(IMU_BODY_X) >= 1 && ABS_AXIS(IMU_BODY_X) <= 3 &&
    ABS_AXIS(IMU_BODY_Y) >= 1 && ABS_AXIS(IMU_BODY_Y) <= 3 &&
    ABS_AXIS(IMU_BODY_Z) >= 1 && ABS_AXIS(IMU_BODY_Z) <= 3 &&
    ABS_AXIS(IMU_BODY_X) != ABS_AXIS(IMU_BODY_Y) && ABS_AXIS(IMU_BODY_X) != ABS_AXIS(IMU_BODY_Z) &&
    ABS_AXIS(IMU_BODY_Y) != ABS_AXIS(IMU_BODY_Z), "IMU axes must be a signed permutation");
_Static_assert(((IMU_BODY_X < 0) + (IMU_BODY_Y < 0) + (IMU_BODY_Z < 0) +
    (ABS_AXIS(IMU_BODY_X) > ABS_AXIS(IMU_BODY_Y)) +
    (ABS_AXIS(IMU_BODY_X) > ABS_AXIS(IMU_BODY_Z)) +
    (ABS_AXIS(IMU_BODY_Y) > ABS_AXIS(IMU_BODY_Z))) % 2 == 0,
    "IMU body axes must be right-handed");

volatile AttitudeData_t attitude_data;
static float q[4], accel[3], bias_sum[3], interval_yaw, interval_dt;
static bool stationary, have_accel, stream_seen, interval_ok;
static uint32_t stream, accel_age;

static float wrap(float x) { return atan2f(sinf(x), cosf(x)); }
static float axis(const float v[3], int n) { return n > 0 ? v[n-1] : -v[-n-1]; }

static void angles(AttitudeData_t *a)
{
    a->roll = atan2f(2.0f*(q[0]*q[1]+q[2]*q[3]), 1.0f-2.0f*(q[1]*q[1]+q[2]*q[2]));
    float s = 2.0f*(q[0]*q[2]-q[3]*q[1]);
    if (s > 1.0f) s = 1.0f;
    if (s < -1.0f) s = -1.0f;
    a->pitch = asinf(s);
    a->yaw = atan2f(2.0f*(q[0]*q[3]+q[1]*q[2]), 1.0f-2.0f*(q[2]*q[2]+q[3]*q[3]));
}

void App_Attitude_Init(void)
{
    attitude_data = (AttitudeData_t){0};
    q[0]=1.0f; q[1]=q[2]=q[3]=0.0f;
    for (unsigned i=0; i<3; ++i) accel[i]=bias_sum[i]=0.0f;
    stationary=have_accel=stream_seen=false;
    interval_yaw=interval_dt=0.0f; interval_ok=false; accel_age=0;
}

void App_Attitude_SetStationary(bool value) { stationary=value; }

void App_Attitude_Process(bool gyro, const float value[3], float dt, uint32_t id)
{
    AttitudeData_t a = attitude_data;
    if (!stream_seen || stream != id) {
        stream_seen=true; stream=id; a.stream_id=id;
        /* Preserve calibrated bias, but never integrate across a missing interval. */
        a.initialized=false; have_accel=false; interval_ok=false;
        interval_yaw=interval_dt=0.0f; a.calibration_samples=0;
        for (unsigned i=0; i<3; ++i) bias_sum[i]=0.0f;
    }
    float v[3] = {axis(value, IMU_BODY_X), axis(value, IMU_BODY_Y), axis(value, IMU_BODY_Z)};
    if (!isfinite(v[0]) || !isfinite(v[1]) || !isfinite(v[2])) {
        interval_ok=false; attitude_data=a; return;
    }
    if (!gyro) {
        for (unsigned i=0; i<3; ++i) accel[i]=v[i];
        have_accel=true; accel_age=0; attitude_data=a; return;
    }
    if (accel_age < 1000U) ++accel_age;
    const float norm = sqrtf(accel[0]*accel[0]+accel[1]*accel[1]+accel[2]*accel[2]);
    const bool gravity_ok = have_accel && accel_age <= 4U && norm > 0.85f*GRAVITY && norm < 1.15f*GRAVITY;
    const float rate = sqrtf(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
    if (!a.calibrated) {
        if (stationary && gravity_ok && rate < 0.15f) {
            for (unsigned i=0; i<3; ++i) bias_sum[i]+=v[i];
            if (++a.calibration_samples >= CAL_SAMPLES) {
                for (unsigned i=0; i<3; ++i) a.gyro_bias[i]=bias_sum[i]/(float)CAL_SAMPLES;
                a.calibrated=true;
                /* Define the relative yaw origin only after bias calibration. */
                a.initialized=false; interval_ok=false;
            }
        } else {
            a.calibration_samples=0;
            for (unsigned i=0; i<3; ++i) bias_sum[i]=0.0f;
        }
    }
    if (!a.initialized && gravity_ok) {
        float roll=atan2f(accel[1], accel[2]);
        float pitch=atan2f(-accel[0], sqrtf(accel[1]*accel[1]+accel[2]*accel[2]));
        float cr=cosf(roll*0.5f), sr=sinf(roll*0.5f), cp=cosf(pitch*0.5f), sp=sinf(pitch*0.5f);
        q[0]=cr*cp; q[1]=sr*cp; q[2]=cr*sp; q[3]=-sr*sp;
        a.initialized=true; angles(&a);
        interval_ok=false; attitude_data=a; return;
    }
    if (!a.initialized || !isfinite(dt) || dt <= 0.0f || dt > 0.05f) {
        interval_ok=false; attitude_data=a; return;
    }
    float w[3] = {v[0]-a.gyro_bias[0], v[1]-a.gyro_bias[1], v[2]-a.gyro_bias[2]};
    if (gravity_ok) {
        /* Measured gravity cross predicted gravity; yaw remains unobservable. */
        float g[3]={2*(q[1]*q[3]-q[0]*q[2]), 2*(q[0]*q[1]+q[2]*q[3]),
                    q[0]*q[0]-q[1]*q[1]-q[2]*q[2]+q[3]*q[3]};
        float m[3]={accel[0]/norm, accel[1]/norm, accel[2]/norm};
        w[0]+=1.0f*(m[1]*g[2]-m[2]*g[1]);
        w[1]+=1.0f*(m[2]*g[0]-m[0]*g[2]);
        w[2]+=1.0f*(m[0]*g[1]-m[1]*g[0]);
    }
    float oldyaw=a.yaw, h=0.5f*dt;
    float n[4]={q[0]+h*(-q[1]*w[0]-q[2]*w[1]-q[3]*w[2]),
                q[1]+h*( q[0]*w[0]+q[2]*w[2]-q[3]*w[1]),
                q[2]+h*( q[0]*w[1]-q[1]*w[2]+q[3]*w[0]),
                q[3]+h*( q[0]*w[2]+q[1]*w[1]-q[2]*w[0])};
    float mag=sqrtf(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]+n[3]*n[3]);
    if (!isfinite(mag) || mag < 0.01f) { a.initialized=false; interval_ok=false; attitude_data=a; return; }
    for (unsigned i=0; i<4; ++i) q[i]=n[i]/mag;
    angles(&a);
    if (a.calibrated) { interval_yaw+=wrap(a.yaw-oldyaw); interval_dt+=dt; }
    else interval_ok=false;
    attitude_data=a;
}

bool App_Attitude_TakeYawDelta(float task_dt, bool continuous, float *delta)
{
    bool valid = delta && continuous && interval_ok && attitude_data.calibrated &&
        isfinite(task_dt) && task_dt > 0.0f && task_dt <= 0.05f &&
        interval_dt > 0.0f && fabsf(interval_dt-task_dt) <= 0.0105f;
    /* Resample average rate to the wheel interval; never apply 2 FIFO periods as 1 wheel period. */
    if (valid) *delta=interval_yaw * task_dt / interval_dt;
    interval_dt=interval_yaw=0.0f;
    interval_ok=true;
    return valid;
}
