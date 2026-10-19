import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import threading
import time  # 引入时间库用于计时

# ================= 配置区 =================
COM_PORT = 'COM13'       # 你的串口号
BAUD_RATE = 921600      # 波特率
MAX_POINTS = 5000       # 画面中保留的最大点数，越多越卡
# ==========================================

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
    print(f"成功打开串口 {COM_PORT}，准备动态绘图...")
except Exception as e:
    print(f"串口打开失败: {e}")
    exit()

# 使用 deque 作为双端队列，可以自动挤出过期的点
xs = deque(maxlen=MAX_POINTS)
ys = deque(maxlen=MAX_POINTS)

# ================= 数据统计相关变量 =================
recv_count = 0
last_time = time.time()
# ====================================================

fig, ax = plt.subplots(figsize=(8, 8))
# s=0.5 画小点，alpha 控制透明度，c='green'
scat = ax.scatter([], [], s=0.5, c='green', alpha=0.8)

# 初始化图表设置
ax.set_xlim(-4000, 4000)
ax.set_ylim(-4000, 4000)
ax.set_aspect('equal')
ax.grid(True, linestyle='--', alpha=0.5)
ax.set_title("Real-time N10 LiDAR PointCloud")
ax.set_xlabel("X (mm)")
ax.set_ylabel("Y (mm)")

def read_serial_data():
    """在后台不断读取串口，每次读空缓冲区"""
    global recv_count, last_time  # 声明使用全局变量
    
    while True:
        try:
            if ser.in_waiting > 0:
                # 一次性读走所有可用行，防止串口数据积压导致延迟
                lines = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').split('\n')
                for line in lines:
                    line = line.strip()
                    if line and ',' in line:
                        parts = line.split(',')
                        try:
                            angle = float(parts[0]) / 100.0
                            dist = float(parts[1])
                            if 40 < dist < 12000:
                                # ================= 核心计数点 =================
                                # 只要成功解析并压入队列，就算作有效接收 1 个点
                                recv_count += 1 
                                # ============================================
                                
                                xs.append(angle)
                                ys.append(dist)
                        except:
                            pass
        except:
            pass
        
        # ================= 每秒打印数据量 =================
        current_time = time.time()
        if current_time - last_time >= 1.0:
            print(f"当前接收数据量: {recv_count} 点/秒")
            recv_count = 0
            last_time = current_time
        # ===================================================

def update(frame):
    """动画更新函数，由 matplotlib 定时调用"""
    # 如果队列里没有数据，直接跳过
    if len(xs) == 0:
        return scat,

    # 获取当前数据
    angles_rad = np.radians(np.array(xs))
    distances = np.array(ys)

    # 极坐标转直角坐标
    x_coords = distances * np.cos(angles_rad)
    y_coords = distances * np.sin(angles_rad)

    # 把新坐标设置给散点图
    points = np.column_stack((x_coords, y_coords))
    scat.set_offsets(points)

    return scat,

# 启动一个后台线程来读串口，不阻塞主线程的界面刷新
thread = threading.Thread(target=read_serial_data, daemon=True)
thread.start()

# 启动动画
# interval=50 表示每 50ms (20Hz) 刷新一次画面，非常流畅
ani = animation.FuncAnimation(fig, update, interval=50, blit=False)

plt.show()

# 关闭窗口后，程序会走到这里退出，关闭串口
ser.close()
print("绘图窗口已关闭，串口已断开。")