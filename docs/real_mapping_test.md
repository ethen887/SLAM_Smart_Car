# 真机栅格地图初步验证

当前瑞萨 3.LSLidar 工程将每个有效 N10 包交给建图模块。STM32 位姿以采样时间戳保留采样间隔，并用 RA 侧串口接收时间估算两板时钟偏移；雷达包使用串口接收中点估算时间，对前后位姿插值，缺少后续位姿时使用最近的较早位姿（最多旧 500 ms）。该方法未进行硬件时钟同步或逐点运动补偿，传感器内部延迟、串口间隔和长期时钟漂移仍会带来误差，需实测确认。

默认地图 100×100 格、0.06 m/格，原点 (-3,-3) m，覆盖 6×6 m；板端以第一帧有效建图位姿为中心，电脑端直接显示 MCU 的闭运算地图快照。原始距离按 mm 转 m，雷达角度按 0.01° 转弧度；0 和 65535 距离被忽略，其他非零距离暂当成命中，需依据雷达协议和实际输出验证过滤规则。雷达安装角和位置偏移默认均为 0；N10 俯视角度顺时针为正，板端将其转换为逆时针为正。可通过 `g_lidar_angle_sign`（+1 或 -1）、`g_lidar_yaw_rad`、`g_lidar_offset_x_m`、`g_lidar_offset_y_m` 调整。停止 CPU 后成组修改，重启会恢复默认值。

在调试器观察 `g_build_map_frames` 是否增长、`g_build_map_skipped_no_pose` 是否变化；`g_build_map_cells` 是地图，启动时清空。停止 CPU 后，可在 GDB 导出完整地图：

```gdb
dump binary memory map.bin &g_build_map_cells[0] &g_build_map_cells[10000]
```

将文件拷贝到电脑后，在仓库根目录运行 `python tools/mapping/render_map_dump.py map.bin map.pgm`；灰色未知、白色空闲、黑色占据。脚本假设 little-endian ARM 的 `Cell` 大小为 12 字节，调试器可用 `p sizeof(Cell)` 验证。图上方向为 X 向右、Y 向上。

雷达和位姿解析器保留跨 DMA 块的不完整帧；队列覆盖及传感器数据质量仍需在硬件验证。无匹配位姿时地图不更新；这不是具有回环校正的 SLAM。
