import time, os, sys
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# 蓝色 LAB 阈值 (必须用实际赛道的A4纸重新校准！)
BLUE_THRESHOLD = [0, 100, -30, 50, -120, -20]

CENTER_X = 320
CENTER_Y = 240

# ================= 串口初始化 =================
fpioa = FPIOA()
fpioa.set_function(40, FPIOA.UART1_TXD)
fpioa.set_function(41, FPIOA.UART1_RXD)
uart = UART(UART.UART1, baudrate=115200, bits=UART.EIGHTBITS, parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)
# ==============================================

sensor_ok = False
display_ok = False

try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(Sensor.VGA)    # 640x480
    sensor.set_pixformat(Sensor.RGB565)
    sensor_ok = True

    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=False)
    display_ok = True

    MediaManager.init()
    sensor.run()
    clock = time.clock()

    last_sent_x = -999
    last_sent_area = -999

    X_CHANGE_THRESHOLD = 3
    AREA_CHANGE_THRESHOLD = 500

    while True:
        os.exitpoint()
        clock.tick()
        img = sensor.snapshot()

        # 绘制中心坐标系
        img.draw_line(0, CENTER_Y, 640, CENTER_Y, color=(128, 128, 128), thickness=1)
        img.draw_line(CENTER_X, 0, CENTER_X, 480, color=(128, 128, 128), thickness=1)
        img.draw_cross(CENTER_X, CENTER_Y, color=(255, 0, 255), size=15, thickness=2)

        # 限定感兴趣区域 (ROI)，建议比赛时只关注画面上半部分和中间，避免地面反光
        track_roi = (0, 0, 640, 400)

        blobs = img.find_blobs([BLUE_THRESHOLD],
                               roi=track_roi,
                               pixels_threshold=800,
                               area_threshold=800,
                               merge=True,
                               margin=25,
                               x_stride=2,
                               y_stride=2)

        valid_blob = None

        if blobs:
            largest_blob = max(blobs, key=lambda b: b.area())

            # 过滤全屏误识别
            if largest_blob.area() < (640 * 480 * 0.8):
                w = largest_blob.w()
                h = largest_blob.h()
                if h > 0:
                    ratio = w / h
                    # 【关键修改】放宽宽高比！因为仰角会导致A4纸变扁
                    # 0.5 到 3.0 的宽高比都可以接受，兼容 A4 竖放且仰视的变形
                    if 0.5 < ratio < 3.0 and largest_blob.elongation() < 0.9:
                        valid_blob = largest_blob

        if valid_blob:
            target_x = valid_blob.cx()
            offset_x = target_x - CENTER_X
            current_area = valid_blob.area()

            # 画框和十字
            img.draw_rectangle(valid_blob.rect(), color=(255, 0, 0), thickness=2)
            img.draw_cross(target_x, valid_blob.cy(), color=(0, 255, 0), size=10, thickness=2)

            # 屏幕上显示，方便标定
            img.draw_string(valid_blob.x(), max(0, valid_blob.y()-20), f"X:{offset_x} A:{current_area}", color=(255,255,255))

            # 判断是否满足发送条件
            x_changed = abs(offset_x - last_sent_x) > X_CHANGE_THRESHOLD
            area_changed = abs(current_area - last_sent_area) > AREA_CHANGE_THRESHOLD

            if x_changed or area_changed or (last_sent_x == -999):
                # 发送 X坐标 和 面积
                uart.write(f"${offset_x},{current_area}\n".encode())
                last_sent_x = offset_x
                last_sent_area = current_area

        else:
            # 没有找到蓝色，不发送数据
            last_sent_x = -999
            last_sent_area = -999

        Display.show_image(img)

except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    error_msg = f"Exception: {e}"
    print(error_msg)
    try:
        with open('/sdcard/error.txt', 'w') as f:
            f.write(error_msg)
    except:
        pass
finally:
    if sensor_ok:
        sensor.stop()
    if display_ok:
        Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
