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
    /* 倾斜静止且陀螺为零时，重力修正只允许进入横滚/俯仰，航向必须保持不变。 */
    for (int i=0;i<50;i++) {
        App_Attitude_Process(false, tilted_accel, 0.01f, 1);
        App_Attitude_Process(true, zero, 0.01f, 1);
    }
    near(attitude_data.yaw,0.0f,0.001f);
    near(attitude_data.accel_forward,0.0f,0.05f);

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
    /* FIFO 一个控制周期内积压两帧时，累计增量要完整交给融合层，不做缩放。 */
    sample(0.01f,-0.02f,1.03f,1); sample(0.01f,-0.02f,1.03f,1);
    assert(App_Attitude_TakeYawDelta(0.01f,true,&d)); near(d,0.02f,0.0001f);
    assert(!App_Attitude_TakeYawDelta(0.01f,true,&d));
    sample(0.01f,-0.02f,1.03f,2);
    assert(!App_Attitude_TakeYawDelta(0.01f,true,&d));
    sample(0.01f,-0.02f,1.03f,2);
    assert(!App_Attitude_TakeYawDelta(0.01f,false,&d));
    sample(0.01f,-0.02f,1.03f,2);
    assert(App_Attitude_TakeYawDelta(0.01f,true,&d));

    odometry_config=(OdometryConfig_t){1.0f/(2*PI),1.0f/(2*PI),1.0f/PI,1,1};
    App_Odometry_Init();
    assert(!App_Odometry_UpdateFusion(0,0,1,true,1.0f,0.0f));

    /* 轮子走 1 m、陀螺给出 0.5 rad：航向以陀螺为主，编码器慢速拉回。 */
    assert(App_Odometry_UpdateFusion(60000,60000,1,true,0.5f,0.0f));
    assert(odometry_data.imu_used);
    const float trust = 1.0f/(1.0f+0.5f*0.5f);
    near(odometry_data.imu_trust,trust,0.0001f);
    const float expected = 0.5f+(1.0f-expf(-1.0f))*(trust*0.8f)*(0.0f-0.5f);
    near(odometry_data.heading,expected,0.0001f);
    near(odometry_data.fused_heading_delta,expected,0.0001f);
    near(odometry_data.heading_delta,0,0.0001f);
    assert(odometry_data.linear_speed > 0.0f);
    assert(odometry_data.x > 0.0f);

    /* IMU 无效时退回纯编码器航向，位置保留。 */
    const float held_x = odometry_data.x;
    assert(App_Odometry_UpdateFusion(120000,120000,1,true,NAN,0.0f));
    assert(!odometry_data.imu_used);
    near(odometry_data.heading,expected,0.0001f);
    near(odometry_data.x,held_x,0.0001f);

    /* 左轮正常、右轮空转：编码器航向突变而陀螺没有，判定打滑并降权。 */
    assert(App_Odometry_UpdateFusion(120000,180000,1,true,0.0f,0.0f));
    assert(odometry_data.imu_trust < 0.5f);
    assert(odometry_data.slip);
    near(odometry_data.heading,expected,0.0001f);
    assert(odometry_data.linear_speed < 1.5f);

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
