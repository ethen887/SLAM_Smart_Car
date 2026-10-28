#include "app_attitude.h"
#include "app_odometry.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

#define PI 3.14159265358979323846f
static void near(float a, float b, float eps) { assert(isfinite(a) && fabsf(a-b)<eps); }
static void sample(float gx, float gy, float gz, uint32_t stream)
{
    const float accel[3]={0,0,9.80665f}, gyro[3]={gx,gy,gz};
    App_Attitude_Process(false, accel, 0.01f, stream);
    App_Attitude_Process(true, gyro, 0.01f, stream);
}

int main(void)
{
    float d=0;
    App_Attitude_Init();
    const float tilted_accel[3]={0, 4.903325f, 8.49281f}, zero[3]={0,0,0};
    App_Attitude_Process(false, tilted_accel, 0.01f, 1);
    App_Attitude_Process(true, zero, 0.01f, 1);
    assert(attitude_data.initialized);
    near(attitude_data.roll,PI/6,0.0001f); near(attitude_data.pitch,0,0.0001f);
    App_Attitude_Init();
    App_Attitude_SetStationary(false);
    for (int i=0;i<250;i++) sample(0.01f,-0.02f,0.03f,1);
    assert(!attitude_data.calibrated);
    App_Attitude_SetStationary(true);
    for (int i=0;i<200;i++) { sample(0.01f,-0.02f,0.03f,1); App_Attitude_TakeYawDelta(0.01f,true,&d); }
    assert(attitude_data.calibrated);
    near(attitude_data.gyro_bias[2],0.03f,0.0001f);
    App_Attitude_SetStationary(false);
    for (int i=0;i<100;i++) {
        sample(0.01f,-0.02f,1.03f,1);
        assert(App_Attitude_TakeYawDelta(0.01f,true,&d));
        near(d,0.01f,0.0001f);
    }
    near(attitude_data.yaw,1.0f,0.002f);
    sample(0.01f,-0.02f,1.03f,1); sample(0.01f,-0.02f,1.03f,1);
    assert(App_Attitude_TakeYawDelta(0.01f,true,&d)); near(d,0.01f,0.0001f);
    assert(!App_Attitude_TakeYawDelta(0.01f,true,&d));
    sample(0.01f,-0.02f,1.03f,2);
    assert(!App_Attitude_TakeYawDelta(0.01f,true,&d));
    sample(0.01f,-0.02f,1.03f,2);
    assert(!App_Attitude_TakeYawDelta(0.01f,false,&d));
    sample(0.01f,-0.02f,1.03f,2);
    assert(App_Attitude_TakeYawDelta(0.01f,true,&d));

    odometry_config=(OdometryConfig_t){1.0f/(2*PI),1.0f/(2*PI),1.0f/PI,1,1};
    odometry_gyro_weight=0.8f;
    App_Odometry_Init();
    assert(!App_Odometry_UpdateImu(0,0,1,true,1));
    assert(App_Odometry_UpdateImu(60000,60000,1,true,0.5f));
    assert(odometry_data.imu_used); near(odometry_data.heading,0.4f,0.0001f);
    near(odometry_data.fused_heading_delta,0.4f,0.0001f);
    near(odometry_data.heading_delta,0,0.0001f);
    assert(App_Odometry_UpdateImu(120000,120000,1,true,NAN));
    assert(!odometry_data.imu_used); near(odometry_data.heading,0.4f,0.0001f);
    OdometryPose_t historical={1,2,PI/2}, map={10,20,PI};
    assert(App_Odometry_SetMapCorrection(map,historical));
    assert(odometry_map_valid);
    OdometryPose_t transform=odometry_map_to_odom;
    near(transform.x,12,0.0001f); near(transform.y,19,0.0001f);
    near(transform.heading,PI/2,0.0001f);
    float local_heading=odometry_data.heading;
    assert(!App_Odometry_SetMapCorrection((OdometryPose_t){NAN,0,0},historical));
    OdometryPose_t output;
    assert(App_Odometry_GetMapPose(&output));
    near(output.x,transform.x-odometry_data.y,0.0001f);
    near(output.y,transform.y+odometry_data.x,0.0001f);
    near(odometry_data.heading,local_heading,0.0001f);
    App_Odometry_Reset(); assert(!App_Odometry_GetMapPose(&output));
    puts("IMU and fused odometry tests passed");
}
