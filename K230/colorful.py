import time, os, sys, gc
import image
from media.sensor import *
from media.display import *
from media.media import *

# 屏幕保持 640x480，这里不需要阈值了，但保留变量以防后续修改
BLUE_THRESHOLD = [0, 100, -128, 127, -128, -24]

try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()

    # 1. 降分辨率保证处理速度
    sensor.set_framesize(Sensor.QVGA)       # 320x240
    sensor.set_pixformat(Sensor.RGB565)     # 保持彩色输入

    # 屏幕保持 640x480
    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=True)

    MediaManager.init()
    sensor.run()
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()

        # 1. 获取彩色图像
        img = sensor.snapshot()

        # 【修改点】删除了 img.binary() 这一步，直接保留彩色画面

        # 2. 内存放大并全屏显示
        try:
            # 将 320x240 的彩色图放大 2 倍铺满屏幕
            scaled_img = img.copy(x_scale=2, y_scale=2)
            Display.show_image(scaled_img)
            scaled_img = None # 用完立刻释放
        except Exception as e:
            print("缩放失败，退回原尺寸显示:", e)
            Display.show_image(img)

        # 释放内存
        img = None
        gc.collect()

except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
