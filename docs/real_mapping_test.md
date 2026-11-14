# 真机栅格地图初步验证

当前瑞萨 3.LSLidar 工程在雷达解析时将每个有效 N10 包交给独立建图模块。STM32 至少有一帧 CRC 正确的位姿后才更新；直接使用最近一次已解析位姿，不执行两板时间同步、超时检测或逐点运动补偿。请先静止建图；移动时位姿延迟、长期漂移和失联后的旧位姿均可能扭曲地图。

默认地图 100×100 格、0.05 m/格，原点 (-2.5,-2.5) m。原始距离按 mm 转 m，雷达角度按 0.01° 转弧度；0 和 65535 距离被忽略，其他非零距离暂当成命中，需依据雷达协议和实际输出验证过滤规则。默认假设雷达角度逆时针为正、零度沿车体前方，安装位置偏移为 0；这些是待实测的假设，可通过 `g_lidar_angle_sign`（+1 或 -1）、`g_lidar_yaw_rad`、`g_lidar_offset_x_m`、`g_lidar_offset_y_m` 调整。停止 CPU 后成组修改，重启会恢复默认值。

在调试器观察 `g_build_map_frames` 是否增长、`g_build_map_skipped_no_pose` 是否变化；`g_build_map_cells` 是地图，启动时清空。停止 CPU 后，可在 GDB 导出完整地图：

```gdb
dump binary memory map.bin &g_build_map_cells[0] &g_build_map_cells[10000]
```

将文件拷贝到电脑后，在仓库根目录运行 `python tools/mapping/render_map_dump.py map.bin map.pgm`；灰色未知、白色空闲、黑色占据。脚本假设 little-endian ARM 的 `Cell` 大小为 12 字节，调试器可用 `p sizeof(Cell)` 验证。图上方向为 X 向右、Y 向上。

当前雷达和位姿仍按独立 DMA 块寻找完整帧，跨块帧可能丢失；队列覆盖及传感器数据质量尚未在硬件验证。无位姿时地图不更新，失联后尚不会自动判断已有位姿过期。不要把当前结果视作已校准的 SLAM。
