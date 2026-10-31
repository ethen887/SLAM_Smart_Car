import serial
import numpy as np
import matplotlib.pyplot as plt
import time
import math
import random

# ================= 配置区 =================
COM_PORT_LIDAR = 'COM8'    # 模拟雷达口 (连 MCU UART1)
COM_PORT_MAP   = 'COM20'   # 地图接收口 (连 MCU UART2)
BAUD_RATE      = 230400    
GRID_SIZE      = 400
TOTAL_BYTES    = 160009    # 11(头) + 160000(数据) + 9(尾)
# ==========================================

# 模拟环境：8米正方形 + 中间2米圆柱
ROOM_SIZE = 8000     
CYLINDER_X = 4000
CYLINDER_Y = 4000
CYLINDER_R = 2000

def get_distance(angle_deg):
    rad = math.radians(angle_deg)
    cos_a, sin_a = math.cos(rad), math.sin(rad)
    dx = abs(ROOM_SIZE / 2 / cos_a) if cos_a != 0 else 999999
    dy = abs(ROOM_SIZE / 2 / sin_a) if sin_a != 0 else 999999
    wall_dist = min(dx, dy)
    
    a, b, c = 1, -2 * (CYLINDER_X * cos_a + CYLINDER_Y * sin_a), CYLINDER_X**2 + CYLINDER_Y**2 - CYLINDER_R**2
    delta = b**2 - 4*a*c
    cyl_dist = 999999
    if delta >= 0:
        t1, t2 = (-b - math.sqrt(delta)) / 2, (-b + math.sqrt(delta)) / 2
        if t1 > 0: cyl_dist = t1
        elif t2 > 0: cyl_dist = t2

    dist = min(wall_dist, cyl_dist) + random.randint(-15, 15)
    return max(40, min(int(dist), 12000))

def send_frame(ser, start_angle_deg):
    frame = bytearray([0xA5, 0x5A, 0x3A, 0x10, 0x46])
    start_angle_val = int(start_angle_deg * 100)
    frame.extend([(start_angle_val >> 8) & 0xFF, start_angle_val & 0xFF])
    
    end_angle_deg = start_angle_deg + (15 * 0.48) 
    for i in range(16):
        pt_angle = start_angle_deg + (i * 0.48)
        dist = get_distance(pt_angle)
        frame.extend([(dist >> 8) & 0xFF, dist & 0xFF, 100])
        
    end_angle_val = int(end_angle_deg * 100)
    frame.extend([(end_angle_val >> 8) & 0xFF, end_angle_val & 0xFF])
    frame.append(sum(frame) & 0xFF)
    ser.write(frame)

# ================= 主流程 =================
try:
    ser_lidar = serial.Serial(COM_PORT_LIDAR, BAUD_RATE, timeout=0.1)
    ser_map   = serial.Serial(COM_PORT_MAP, BAUD_RATE, timeout=0.1, write_timeout=2.0)
    print("串口全部打开成功！")
except Exception as e:
    print(f"串口打开失败: {e}")
    exit()

# 1. 雷达扫描 5 秒（加入延时，防止串口拥挤）
print("开始发送模拟雷达数据... (持续 5 秒)")
current_angle = 0.0
start_time = time.time()
frame_count = 0

while time.time() - start_time < 5.0:
    try:
        send_frame(ser_lidar, current_angle)
        current_angle = (current_angle + 7.68) % 360.0
        frame_count += 1
        time.sleep(0.005)  # ★ 关键修改：每帧延时 5ms，防止数据洪峰把 MCU 打爆
    except:
        pass
print(f"雷达数据发送完毕，共发送 {frame_count} 帧。等待 MCU 建图...")
time.sleep(1) # 给 MCU 1 秒钟把剩下的点算完

# 2. 请求 160KB 大地图
print("发送 @map 指令，准备接收 160KB 栅格地图...")
ser_map.reset_input_buffer()
ser_map.write(b'@map')

# 3. 接收数据
data = bytearray()
start_recv = time.time()
print("正在接收数据，请稍候...")
while len(data) < TOTAL_BYTES and time.time() - start_recv < 20.0:
    if ser_map.in_waiting:
        data.extend(ser_map.read(ser_map.in_waiting))
        
# 4. 解析与画图
print(f"接收完毕，总共收到 {len(data)} 字节。")
start_idx = data.find(b'MAP_START\r\n')
if start_idx != -1 and len(data) >= start_idx + TOTAL_BYTES:
    payload = data[start_idx + 11 : start_idx + 11 + 160000]
    grid = np.frombuffer(payload, dtype=np.int8)
    grid = grid.reshape((GRID_SIZE, GRID_SIZE))

    # 画图
    fig, ax = plt.subplots(figsize=(10, 10), facecolor='lightgray')
    from matplotlib.colors import ListedColormap
    cmap = ListedColormap(['white', 'lightgray', 'black'])
    
    grid_display = np.ones_like(grid) # 默认未知(灰)
    grid_display[grid < 0] = 0        # 空闲(白)
    grid_display[grid > 10] = 2       # 障碍(黑)

    ax.imshow(grid_display, cmap=cmap, origin='lower', interpolation='nearest', vmin=0, vmax=2)
    
    ax.grid(True, which='both', color='darkgray', linestyle='-', linewidth=0.5, alpha=0.7)
    ax.set_xticks(np.arange(0, GRID_SIZE, 20))
    ax.set_yticks(np.arange(0, GRID_SIZE, 20))
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.plot(GRID_SIZE//2, GRID_SIZE//2, 'r+', markersize=12, markeredgewidth=2)
    ax.set_title("SLAM Grid Map (160KB Array Mode)", fontsize=16)
    plt.show()
else:
    print(f"未接收到完整的 160KB 数组，可能是丢包了。目前收到 {len(data)} 字节，期望 {TOTAL_BYTES} 字节。")

ser_lidar.close()
ser_map.close()