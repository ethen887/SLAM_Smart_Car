# IMU 姿态、里程计与后续 SLAM 对接

## 当前实现

- 坐标约定：车体 X 前、Y 左、Z 上，右手系；里程计起始车头为 odom X，向左为 odom Y，逆时针航向为正。距离使用 m，角度使用 rad，时间使用 s。
- `app_attitude.c` 按 FIFO 顺序读取原始 SI 单位样本，用四元数积分三轴陀螺仪；接近 1g 时用加速度修正横滚和俯仰，修正量只进入陀螺的 X/Y 轴，Z 轴不修正，避免倾斜时把加速度噪声灌进航向。六轴 IMU 无法观测绝对航向，因此 yaw 是相对角度，会随时间漂移。无硬件磁力计，不应把 yaw 当成罗盘值。
- 开机必须让轮子及车体静止约 2 秒，累计 200 个满足静止和重力条件的陀螺仪样本后计算零偏。校准前里程计仍以编码器运行；移动或重力异常会重新开始计数。完成校准时 yaw 从零开始。此后只要判定静止，零偏会以约 5 秒时间常数继续跟踪，温漂不再逐点累积。FIFO 数据流中断恢复后姿态重新依据重力初始化，yaw 的参考会重置，但校准零偏保留；不会跨中断积分。
- `attitude_data.accel_forward` 为车体前向运动加速度：加速度读数减去重力在机体 X 上的投影，再经慢速低通，供速度预测使用。它不参与横滚/俯仰之外的角度修正。
- 当前装配为传感器 Y 轴指向车头、X 轴指向车体左侧，故右手系下 Z 轴朝下。`app_attitude.c` 的 `IMU_BODY_X/Y/Z` 因此设为 `2 / 1 / -3`：传感器 Y 映射到车体前、X 映射到车体左、Z 取反映射到车体上。三者的绝对值必须各用一次 1、2、3 并构成右手系，加速度和陀螺仪共用此转换。车体坐标始终是 X 前、Y 左、Z 上，与编码器里程计一致。以车体水平静止时 Z 接近 +9.81 m/s²、逆时针水平旋转时 Z 角速度为正、向前推车时 `accel_forward` 为正进行检查。更换安装后必须同步修改这三个宏，否则姿态和里程计都会错。
- `app_odometry.c` 输出连续本地 odom 位姿，航向和速度分工如下：
  - 航向以陀螺积分为主：`fused_heading_delta` 先取 IMU 增量，再用编码器累计航向做时间常数 `odometry_heading_tau_s`（默认 1 s）的慢速拉回。编码器只负责抑制陀螺长期漂移，不再直接主导转角。
  - 速度用 IMU 加速度外推，再用轮速按 `odometry_speed_tau_s`（默认 0.15 s）校正；位置按融合速度沿融合航向积分。
  - 编码器可信度 `imu_trust` 由两项相乘：轮式角速度与陀螺角速度之差（阈值 `odometry_slip_rate_thr`）、轮速加速度与 `accel_forward` 之差（阈值 `odometry_slip_accel_thr`）。差值越大可信度越低，打滑时航向和速度都自动退回 IMU。
  - 轮式与陀螺的累计航向差超过 `odometry_max_heading_error`（默认 0.6 rad）时，认为编码器已明显打滑，直接重建轮式航向参考，而不是把航向硬拉过去。
  - 轮子不动且 IMU 安静时执行零速修正，连续 3 个周期后把融合速度清零，静止不再累积位置漂移。
- `heading_delta` 仍是原始编码器转角，`distance_delta` 仍是纯编码器距离；`wheel_speed`、`imu_trust`、`slip`、`imu_used` 用于在调试器中判断当前融合状态。
- 当未校准、FIFO 错误/积压、周期过长或数据无效时，该周期使用纯编码器航向和轮速。104 Hz IMU 与约 100 Hz 轮速并不同步，`App_Attitude_TakeYawDelta` 把 FIFO 一个周期内累计的全部真实增量交给融合层，不再按控制周期缩放；FIFO 未提供精确采样时间戳，时间同步仍是当前阶段的近似。
- 打滑时位置短时由 IMU 推算，但六轴没有绝对速度或位置参考，持续打滑仍会漂移；本融合不解决轮胎打滑的物理根源，只降低其对航向和短时位置的影响。轮半径、轮距和方向来自当前实验配置，必须实测核对。

## 板上检查

1. 固定好 IMU，核对轴向和编码器方向，断电后重启并保持静止。观察 `lsm6dsr_init_result`、`lsm6dsr_fifo_debug.status`、`attitude_data.calibration_samples/calibrated`，等待校准完成。
2. 手动小角度倾斜车体并检查 `attitude_data.roll/pitch` 的方向；前后推车检查 `accel_forward` 的符号和幅值。再低速原地左转和右转，核对 `attitude_data.yaw` 和 `odometry_data.heading` 的正负；同时观察 `imu_used`、`imu_trust`、`slip`、`wheel_speed` 和 `fused_heading_delta`。无法确认方向前不要使用自主运动。
3. 直线行驶时 `imu_trust` 应接近 1；单侧轮子空转或急加速打滑时应明显下降并置位 `slip`。若正常行驶也长期偏低，先核对轮径、轮距和 IMU 轴向，而不是盲目调阈值。
4. 检查 IMU FIFO 是否经常报错、溢出或积压；检查 `move_task_dt_s`、控制任务栈高水位和系统稳定性。姿态计算在运动任务执行，增加了计算和栈开销，目标板的时序和栈余量仍需实际测量。
5. 移动期间断开或模拟 IMU 失败时应看到 `imu_used=false` 且位姿仍由编码器更新；重新恢复后不应把丢失的陀螺仪积分补入。绝对精度应以后通过独立距离/角度测量标定。
6. `tools/odometry/run_straight.py` 的 `motion.csv` 已包含 `imu_trust`、`slip`、`accel_forward`、`fused_heading_delta` 四个融合诊断字段，可直接用于打滑实验的离线分析，不必只靠调试器在线观察。改动记录结构后必须重新烧录固件，旧日志无法混合解析。

## 后续 UART / SLAM 约定

当前仅准备了融合结果和纠正 API，**没有实现板间 UART 收发或指定协议**。现有 USART1 为 PB6 TX、PB7 RX、115200 8N1；是否用于板间通信、引脚接线和上层接口需两板共同确定。不可把调试发布时刻当作 IMU 硬件采样时刻。

保留 `odom` 为连续局部坐标系：上层发送某时刻的地图位姿 `map_pose`，同时提供该时刻对应的 `odom_pose_at_measurement`；在运动任务调用 `App_Odometry_SetMapCorrection(map_pose, odom_pose_at_measurement)`，计算 `map_T_odom = map_T_body * inverse(odom_T_body)`。`App_Odometry_GetMapPose` 再将最新 odom 位姿变换到地图坐标。历史位姿和地图位姿必须对应同一时刻；不能将延迟到达的 SLAM 位姿与收到时的当前 odom 位姿配对，否则会引入延迟误差。重置 odom 会清除此变换，需重新建立。

未来 UART 帧至少应包含版本、类型、长度、序列号、时间基准/时间戳、数据有效状态、校验，并明确小端及数值单位。上行建议携带连续 odom 位姿、速度、IMU 状态和采集时间；下行携带 SLAM 位姿及其对应的 odom 时间/位姿，并处理乱序、超时、复位及时钟同步。中断只负责缓存数据，解析与调用纠正 API 放在运动任务或通过受保护通道交接；当前各接口仅保证同一任务使用，跨任务读取结构体需同步。未来通信代码必须处理 DMA/中断发送缓冲区的所有权，不能在发送未完成时覆盖缓冲区。
