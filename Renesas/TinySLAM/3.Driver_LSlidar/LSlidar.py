import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
from collections import deque
import threading
import time

# ================= 串口配置 =================
COM_PORT      = 'COM13'
BAUD_RATE     = 115200
SEND_INTERVAL = 2.0        # 每 2 秒发一次 @map
MAX_POINTS    = 5000       # 一帧最多保留点数
# ============================================

# ================= 栅格建图配置（全部 cm） =================
GRID_RES_CM = 5            # 每格 5cm
GRID_SIZE   = 400          # 400x400 格 = 20m x 20m
GRID_ORIGIN = 200          # 车在栅格中的位置（中心）

L_OCC  = 0.85              # 占据更新量（正）
L_FREE = -0.40             # 空闲更新量（负）
L_MIN  = -5.0
L_MAX  =  5.0

log_odds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
# ============================================================

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
    print(f"成功打开串口 {COM_PORT} @ {BAUD_RATE}")
except Exception as e:
    print(f"串口打开失败: {e}")
    exit()

ser.reset_input_buffer()

xs = deque(maxlen=MAX_POINTS)
ys = deque(maxlen=MAX_POINTS)

frame_x = []   # 存 cm 坐标
frame_y = []

# ================= 数据统计 =================
recv_count  = 0
raw_bytes   = 0
ok_count    = 0
frame_count = 0
last_send   = 0.0
last_stat   = time.time()
# ============================================

# ================= 图形 =================
fig, (ax_cloud, ax_grid) = plt.subplots(1, 2, figsize=(16, 8))

# ---------- 左：点云（单位 cm） ----------
scat = ax_cloud.scatter([], [], s=0.5, c='green', alpha=0.8)
ax_cloud.set_xlim(-400, 400)   # ±4m = ±400cm
ax_cloud.set_ylim(-400, 400)
ax_cloud.set_aspect('equal')
ax_cloud.grid(True, linestyle='--', alpha=0.5)
ax_cloud.set_title("Real-time N10 LiDAR PointCloud (refresh every 2s, cm)")
ax_cloud.set_xlabel("X (cm)")
ax_cloud.set_ylabel("Y (cm)")

# ---------- 右：栅格地图（单位 cm） ----------
extent = [
    -GRID_ORIGIN * GRID_RES_CM, (GRID_SIZE - GRID_ORIGIN) * GRID_RES_CM,
    -GRID_ORIGIN * GRID_RES_CM, (GRID_SIZE - GRID_ORIGIN) * GRID_RES_CM,
]
im = ax_grid.imshow(
    np.full((GRID_SIZE, GRID_SIZE), 0.5),
    cmap='gray', vmin=0.0, vmax=1.0,
    extent=extent, origin='lower', interpolation='nearest'
)
ax_grid.set_xlim(-500, 500)
ax_grid.set_ylim(-500, 500)
ax_grid.set_aspect('equal')
ax_grid.set_title("Occupancy Grid Map (5cm/cell, car at origin)")
ax_grid.set_xlabel("X (cm)")
ax_grid.set_ylabel("Y (cm)")
ax_grid.plot(0, 0, 'ro', markersize=8, label='car (0,0)')
ax_grid.legend(loc='upper right')
# ============================================

def log_odds_to_prob(lo):
    return 1.0 - 1.0 / (1.0 + np.exp(lo))

def bresenham_line(x0, y0, x1, y1):
    points = []
    dx = abs(x1 - x0); sx = 1 if x0 < x1 else -1
    dy = -abs(y1 - y0); sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        points.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy; x0 += sx
        if e2 <= dx:
            err += dx; y0 += sy
    return points

def update_grid_from_frame(px_list, py_list):
    """px_list/py_list 单位已经是 cm"""
    global log_odds
    car_gx, car_gy = GRID_ORIGIN, GRID_ORIGIN

    for px_cm, py_cm in zip(px_list, py_list):
        gx = int(px_cm / GRID_RES_CM) + GRID_ORIGIN
        gy = int(py_cm / GRID_RES_CM) + GRID_ORIGIN

        if not (0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE):
            continue

        line = bresenham_line(car_gx, car_gy, gx, gy)

        for (lx, ly) in line[:-1]:
            if 0 <= lx < GRID_SIZE and 0 <= ly < GRID_SIZE:
                log_odds[ly, lx] = np.clip(
                    log_odds[ly, lx] + L_FREE, L_MIN, L_MAX
                )

        log_odds[gy, gx] = np.clip(
            log_odds[gy, gx] + L_OCC, L_MIN, L_MAX
        )

def read_serial_data():
    global recv_count, raw_bytes, ok_count, frame_count
    global last_send, last_stat
    global xs, ys, frame_x, frame_y

    while True:
        now = time.time()

        # 每 2 秒发一次 @map
        if now - last_send >= SEND_INTERVAL:
            try:
                ser.write(b'@map')
                print(f"[{time.strftime('%H:%M:%S')}] >>> 发送 @map")
            except Exception:
                pass
            last_send = now

        # 读串口
        try:
            n = ser.in_waiting
            if n > 0:
                chunk = ser.read(n)
                raw_bytes += len(chunk)
                text = chunk.decode('utf-8', errors='ignore')

                for line in text.split('\n'):
                    line = line.strip()
                    if not line:
                        continue

                    if line == 'ok':
                        ok_count += 1
                        print(f"  >>> 收到 ok (累计 {ok_count})")
                        continue

                    if 'BOOT' in line:
                        print("  >>> 收到 BOOT")
                        continue

                    # 结束标记：提交一帧 + 建图
                    if line == 'S':
                        if len(frame_x) > 0:
                            xs = deque(frame_x, maxlen=MAX_POINTS)
                            ys = deque(frame_y, maxlen=MAX_POINTS)
                            recv_count = len(frame_x)
                            frame_count += 1
                            print(f"  >>> 收到完整帧 #{frame_count}，{recv_count} 点，开始建图")
                            update_grid_from_frame(frame_x, frame_y)
                        frame_x = []
                        frame_y = []
                        continue

                    # 点云行：兼容两列 / 四列，统一转成 cm
                    if ',' in line:
                        parts = line.split(',')
                        try:
                            if len(parts) == 2:
                                # angle(0.01度), dist(mm)
                                angle_deg = float(parts[0]) / 100.0
                                dist_mm   = float(parts[1])
                                if 40 < dist_mm < 12000:
                                    dist_cm = dist_mm / 10.0
                                    a = np.radians(angle_deg)
                                    frame_x.append(dist_cm * np.cos(a))
                                    frame_y.append(dist_cm * np.sin(a))
                            elif len(parts) == 4:
                                # px(mm), py(mm), angle(0.01度), dist(mm)
                                px_mm     = float(parts[0])
                                py_mm     = float(parts[1])
                                angle_deg = float(parts[2]) / 100.0
                                dist_mm   = float(parts[3])
                                if 40 < dist_mm < 12000:
                                    px_cm   = px_mm / 10.0
                                    py_cm   = py_mm / 10.0
                                    dist_cm = dist_mm / 10.0
                                    a = np.radians(angle_deg)
                                    frame_x.append(px_cm + dist_cm * np.cos(a))
                                    frame_y.append(py_cm + dist_cm * np.sin(a))
                        except Exception:
                            pass
        except Exception:
            pass

        # 每秒统计
        now2 = time.time()
        if now2 - last_stat >= 1.0:
            print(f"[统计] 原始字节: {raw_bytes}/s | ok 累计: {ok_count} | 帧累计: {frame_count}")
            raw_bytes = 0
            last_stat = now2

        time.sleep(0.005)

def update(frame):
    # 左图：点云（cm）
    if len(xs) > 0:
        points = np.column_stack((np.array(xs), np.array(ys)))
        scat.set_offsets(points)

    # 右图：栅格（log-odds → 概率）
    prob = log_odds_to_prob(log_odds)
    im.set_data(prob)

    return scat, im

thread = threading.Thread(target=read_serial_data, daemon=True)
thread.start()

ani = animation.FuncAnimation(fig, update, interval=100, blit=False)
plt.tight_layout()
plt.show()

ser.close()
print("绘图窗口已关闭，串口已断开。")