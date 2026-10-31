# import serial
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import numpy as np
# from collections import deque
# import threading
# import time

# # ================= 配置区 =================
# COM_PORT = 'COM13'       # 你的串口号
# BAUD_RATE = 921600       # ← 先降到 115200 和 MCU 对齐；跑通后再回 921600
# MAX_POINTS = 5000        # 画面中保留的最大点数
# # ==========================================

# try:
#     ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.01)
#     print(f"成功打开串口 {COM_PORT} @ {BAUD_RATE}，准备动态绘图...")
# except Exception as e:
#     print(f"串口打开失败: {e}")
#     exit()

# # 清掉上电残留
# ser.reset_input_buffer()

# # 主动发一次 @map，触发 MCU 回 ok
# ser.write(b'@map')
# print("已发送 @map，等待 MCU 回应...")

# xs = deque(maxlen=MAX_POINTS)
# ys = deque(maxlen=MAX_POINTS)

# # ================= 数据统计 =================
# recv_count = 0          # 每秒解析出的有效点数
# raw_bytes = 0           # 每秒收到的原始字节数
# ok_count = 0            # 收到多少次 "ok"
# last_time = time.time()
# # ============================================

# fig, ax = plt.subplots(figsize=(8, 8))
# scat = ax.scatter([], [], s=0.5, c='green', alpha=0.8)

# ax.set_xlim(-4000, 4000)
# ax.set_ylim(-4000, 4000)
# ax.set_aspect('equal')
# ax.grid(True, linestyle='--', alpha=0.5)
# ax.set_title("Real-time N10 LiDAR PointCloud")
# ax.set_xlabel("X (mm)")
# ax.set_ylabel("Y (mm)")

# def read_serial_data():
#     """后台线程：读串口，解析 ok / 点云行"""
#     global recv_count, raw_bytes, ok_count, last_time

#     while True:
#         try:
#             if ser.in_waiting > 0:
#                 chunk = ser.read(ser.in_waiting)
#                 raw_bytes += len(chunk)

#                 # 原样打印，方便看 BOOT / ok 到底有没有来
#                 text = chunk.decode('utf-8', errors='ignore')
#                 if text.strip():
#                     print(f"[RX] {text!r}")

#                 for line in text.split('\n'):
#                     line = line.strip()
#                     if not line:
#                         continue

#                     # ---- 匹配 ok ----
#                     if line == 'ok':
#                         ok_count += 1
#                         print(f"  >>> 收到 ok (累计 {ok_count})")
#                         continue

#                     # ---- 匹配 BOOT ----
#                     if 'BOOT' in line:
#                         print("  >>> 收到 BOOT，MCU 已启动")
#                         continue

#                     # ---- 匹配点云行：x,y,angle,dist 四列 ----
#                     if ',' in line:
#                         parts = line.split(',')
#                         try:
#                             if len(parts) == 4:
#                                 # MCU 发的是 pos_x,pos_y,angle,dist
#                                 px = float(parts[0])
#                                 py = float(parts[1])
#                                 angle_deg = float(parts[2]) / 100.0
#                                 dist = float(parts[3])
#                                 if 40 < dist < 12000:
#                                     recv_count += 1
#                                     # 极坐标转直角，再叠加车位置
#                                     a = np.radians(angle_deg)
#                                     xs.append(px + dist * np.cos(a))
#                                     ys.append(py + dist * np.sin(a))
#                             elif len(parts) == 2:
#                                 # 兼容 2 列格式：angle,dist
#                                 angle_deg = float(parts[0]) / 100.0
#                                 dist = float(parts[1])
#                                 if 40 < dist < 12000:
#                                     recv_count += 1
#                                     a = np.radians(angle_deg)
#                                     xs.append(dist * np.cos(a))
#                                     ys.append(dist * np.sin(a))
#                         except Exception:
#                             pass
#         except Exception:
#             pass

#         # ================= 每秒打印统计 =================
#         current_time = time.time()
#         if current_time - last_time >= 1.0:
#             print(f"[统计] 有效点: {recv_count}/s | 原始字节: {raw_bytes}/s | ok: {ok_count}")
#             recv_count = 0
#             raw_bytes = 0
#             last_time = current_time
#         # ===============================================

# def update(frame):
#     """动画更新"""
#     if len(xs) == 0:
#         return scat,
#     points = np.column_stack((np.array(xs), np.array(ys)))
#     scat.set_offsets(points)
#     return scat,

# thread = threading.Thread(target=read_serial_data, daemon=True)
# thread.start()

# ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
# plt.show()

# ser.close()
# print("绘图窗口已关闭，串口已断开。")

import serial
import time

# ================= 配置区 =================
COM_PORT  = 'COM13'    # 改成你实际的串口号
BAUD_RATE = 921600     # 和 MCU 的 g_uart2 保持一致
SEND_INTERVAL = 2.0    # 每 2 秒发一次 @map
# ==========================================

try:
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.05)
    print(f"串口 {COM_PORT} @ {BAUD_RATE} 已打开")
except Exception as e:
    print(f"串口打开失败: {e}")
    exit()

# 清掉上电残留
ser.reset_input_buffer()

rx_total = 0
last_send = 0.0

print("开始循环：每 2 秒发送一次 '@map'，收到的所有字节会原样打印。Ctrl+C 退出。\n")

try:
    while True:
        now = time.time()

        # ---- 每 2 秒发一次 @map ----
        if now - last_send >= SEND_INTERVAL:
            ser.write(b'@map')
            print(f"[{time.strftime('%H:%M:%S')}] >>> 发送 @map")
            last_send = now

        # ---- 把串口里此刻能读到的字节全读出来 ----
        n = ser.in_waiting
        if n > 0:
            data = ser.read(n)
            rx_total += len(data)
            # !r 让 \r \n 等不可见字符也显示出来
            print(f"           <<< 收到 {len(data)} 字节: {data!r}")

            # 顺便把可打印文本单独打出来，方便看 ok / BOOT
            text = data.decode('utf-8', errors='ignore').strip()
            if text:
                print(f"           <<< 文本: {text}")

        time.sleep(0.01)

except KeyboardInterrupt:
    print(f"\n退出。共收到 {rx_total} 字节。")
finally:
    ser.close()
    print("串口已关闭。")