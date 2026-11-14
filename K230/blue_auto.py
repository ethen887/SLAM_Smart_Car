import time, os, sys, math
import image

from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# =====================================================================================================
#                                        ↓↓↓ 可调参数区 ↓↓↓
# =====================================================================================================

# ------------------------------ 1. 颜色阈值 (LAB) ------------------------------
BLUE_THRESHOLD = [15, 60, 14, 34, -128, -38]

# ------------------------------ 2. 感兴趣区域 ROI ------------------------------
TRACK_ROI = (0, 0, 640, 480)

# ------------------------------ 3. blob 粗筛参数 ------------------------------
BLOB_PIXELS_THRESHOLD = 150   # 最少像素数
BLOB_AREA_THRESHOLD   = 150   # 最小面积
BLOB_MERGE            = True
BLOB_MARGIN           = 25
BLOB_X_STRIDE         = 2
BLOB_Y_STRIDE         = 2

# ------------------------------ 4. 目标形状过滤 ------------------------------
ASPECT_MIN            = 0.05
ASPECT_MAX            = 2.5

# ------------------------------ 5. 串口参数 ------------------------------
UART_TX_PIN = 40
UART_RX_PIN = 41
UART_BAUD   = 115200
SEND_X_CHANGE    = 3          # X像素变化超过 3 个像素才发送
SEND_AREA_CHANGE = 500        # 面积变化超过 500 个像素才发送

# ------------------------------ 6. 显示 / 调试开关 ------------------------------
TO_IDE            = False      # 调参 True，跑车 False
SHOW_FPS          = True
SHOW_REJECT       = True

# =====================================================================================================
#                                        ↑↑↑ 可调参数区 ↑↑↑
# =====================================================================================================

CENTER_X = 320
CENTER_Y = 240

# ========================= 主程序 =========================

sensor = None
uart = None

try:
    # ---------------- 串口初始化 ----------------
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUD, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    # ---------------- 摄像头 ----------------
    sensor = Sensor(width=1280, height=960)
    sensor.reset()

    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    sensor.set_framesize(Sensor.VGA)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=TO_IDE)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # ================= 新增：等待 RA8P1 发送启动指令 =================
    print("Waiting for 'camera_ok' from RA8P1...")

    # 清空一下之前可能存在的串口残留数据
    if uart.any():
        uart.read()

    # 阻塞等待，直到收到 "camera_ok"
    while True:
        os.exitpoint()
        if uart.any():
            cmd = uart.read(20)
            if cmd and b"camera_ok" in cmd:
                print("Received 'camera_ok', starting vision task!")
                break
        time.sleep_ms(10)  # 避免死循环跑满 CPU
    # ==============================================================

    last_sent_x = -999
    last_sent_area = -999
    fps = 0
    fps_t = time.ticks_ms()

    while True:
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot()

        # 绘制中心坐标系
        img.draw_line(0, CENTER_Y, 640, CENTER_Y, color=(128, 128, 128), thickness=1)
        img.draw_line(CENTER_X, 0, CENTER_X, 480, color=(128, 128, 128), thickness=1)
        img.draw_cross(CENTER_X, CENTER_Y, color=(255, 0, 255), size=15, thickness=2)

        blobs = img.find_blobs([BLUE_THRESHOLD],
                               roi=TRACK_ROI,
                               pixels_threshold=BLOB_PIXELS_THRESHOLD,
                               area_threshold=BLOB_AREA_THRESHOLD,
                               merge=BLOB_MERGE,
                               margin=BLOB_MARGIN,
                               x_stride=BLOB_X_STRIDE,
                               y_stride=BLOB_Y_STRIDE)

        valid_blob = None
        reject_reason = "no_blob"
        blob_num = 0

        if blobs:
            blob_num = len(blobs)

            # 画灰色框
            for b in blobs:
                if b.area() < (640 * 480 * 0.8):
                    img.draw_rectangle(b.rect(), color=(150, 150, 150), thickness=1)

            blobs.sort(key=lambda b: b.area(), reverse=True)
            largest_blob = blobs[0]

            if largest_blob.area() < (640 * 480 * 0.8):
                w = largest_blob.w()
                h = largest_blob.h()
                if h > 0:
                    ratio = w / h
                    if ASPECT_MIN < ratio < ASPECT_MAX:
                        valid_blob = largest_blob
                    else:
                        reject_reason = "aspect"
                else:
                    reject_reason = "invalid_size"

        # ================== 处理并发送数据 ==================
        if valid_blob:
            target_x = valid_blob.cx()
            # 计算X像素偏差 (左负右正)
            offset_x_pixel = target_x - CENTER_X
            # 计算当前的像素面积
            current_area = valid_blob.area()

            # 画出红色粗框和绿色十字
            img.draw_rectangle(valid_blob.rect(), color=(255, 0, 0), thickness=3)
            img.draw_cross(target_x, valid_blob.cy(), color=(0, 255, 0), size=10, thickness=2)

            # 屏幕显示：X像素 和 面积像素
            img.draw_string(10, 10, "X:%d A:%d" % (offset_x_pixel, current_area), color=(255, 255, 0), scale=3.0)

            # 串口死区发送判断
            x_changed = abs(offset_x_pixel - last_sent_x) > SEND_X_CHANGE
            area_changed = abs(current_area - last_sent_area) > SEND_AREA_CHANGE

            if x_changed or area_changed or (last_sent_x == -999):
                # 发送格式：$X像素,面积像素\n (例如：$-15,45000)
                uart_str = "$%d,%d\n" % (offset_x_pixel, current_area)
                uart.write(uart_str.encode())

                last_sent_x = offset_x_pixel
                last_sent_area = current_area
        else:
            last_sent_x = -999
            last_sent_area = -999

        # 屏幕底部显示调试信息
        if SHOW_REJECT:
            img.draw_string(10, 440, "blobs:%d rej:%s" % (blob_num, reject_reason), color=(255, 255, 255), scale=2.0)
        if SHOW_FPS:
            img.draw_string(10, 400, "fps:%d" % fps, color=(255, 255, 255), scale=2.0)

        Display.show_image(img)

        now = time.ticks_ms()
        if now - fps_t >= 1000:
            fps = int(clock.fps())
            fps_t = now

        # ★ 核心修改点：强制加上 15ms 的延时，把发送频率限制在 60Hz 左右，防止串口冲爆单片机！
        time.sleep_ms(15)

except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
    try:
        with open('/sdcard/error.txt', 'w') as f:
            f.write("Exception: %s" % e)
    except Exception:
        pass
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
