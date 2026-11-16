#import time, os, sys, gc
#import image
#from media.sensor import *
#from media.display import *
#from media.media import *

## LAB蓝色阈值
#BLUE_THRESHOLD = [0, 100, -3, 127, -128, -13]

## 过滤过小的白色区域
#MIN_BLOB_AREA = 10

## 描边颜色
#EDGE_COLOR = (255, 0, 0)

#try:
#    sensor = Sensor(width=1280, height=960)
#    sensor.reset()

#    sensor.set_framesize(Sensor.QVGA)       # 320x240
#    sensor.set_pixformat(Sensor.RGB565)     # 彩色输入

#    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=True)

#    MediaManager.init()
#    sensor.run()
#    clock = time.clock()

#    while True:
#        os.exitpoint()
#        clock.tick()

#        # 1. 图像获取与二值化
#        raw_img = sensor.snapshot()
#        display_img = raw_img.copy()
#        display_img.binary([BLUE_THRESHOLD])

#        # 2. 转灰度图：找色块 + 边缘检测都用灰度图，格式统一
#        gray_img = display_img.to_grayscale(copy=True)

#        # 形态学平滑，减少内部噪点
#        try:
#            gray_img.open(1)
#        except:
#            pass

#        # ✅ 修正：在灰度图上找白色色块，阈值格式正确
#        blobs = gray_img.find_blobs([(255,255)], pixels_threshold=MIN_BLOB_AREA, area_threshold=MIN_BLOB_AREA, merge=True)

#        # 调试打印：串口看有没有找到色块、面积多大
#        print(f"找到色块数量: {len(blobs)}")

#        if blobs:
#            blobs.sort(key=lambda b: b.area(), reverse=True)
#            b = blobs[0]
#            rx, ry, rw, rh = b.rect()
#            print(f"最大色块面积: {b.area()}, 矩形: {rx},{ry},{rw},{rh}")

#            # 3. 边缘检测（灰度图直接做，不用再转一次）
#            gray_img.find_edges(image.EDGE_SIMPLE, threshold=(100, 255))

#            # 4. 只在色块矩形范围内描绘边沿
#            step = 1
#            for y in range(ry, ry + rh, step):
#                for x in range(rx, rx + rw, step):
#                    if gray_img.get_pixel(x, y) > 200:
#                        display_img.draw_circle(x, y, 2, color=EDGE_COLOR, fill=True)

#            # 5. 边缘断裂补全逻辑（贴边时补线）
#            if rx <= 2:
#                display_img.draw_line(0, ry, 0, ry + rh, color=EDGE_COLOR, thickness=2)
#            if ry <= 2:
#                display_img.draw_line(rx, 0, rx + rw, 0, color=EDGE_COLOR, thickness=2)
#            if rx + rw >= 318:
#                display_img.draw_line(319, ry, 319, ry + rh, color=EDGE_COLOR, thickness=2)
#            if ry + rh >= 238:
#                display_img.draw_line(rx, 239, rx + rw, 239, color=EDGE_COLOR, thickness=2)

#        # 6. 缩放全屏显示
#        try:
#            scaled_img = display_img.copy(x_scale=2, y_scale=2)
#            Display.show_image(scaled_img)
#            scaled_img = None
#        except Exception:
#            Display.show_image(display_img)

#        raw_img = None
#        display_img = None
#        gray_img = None
#        gc.collect()
#        print("FPS:", clock.fps())

#except KeyboardInterrupt as e:
#    print("user stop: ", e)
#except BaseException as e:
#    print(f"Exception {e}")
#finally:
#    if isinstance(sensor, Sensor):
#        sensor.stop()
#    Display.deinit()
#    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
#    time.sleep_ms(100)
#    MediaManager.deinit()

import time, os, sys, gc
import image
from media.sensor import *
from media.display import *
from media.media import *

# LAB蓝色阈值
BLUE_THRESHOLD = [0, 100, -3, 127, -128, -13]

# 过滤过小的白色区域
MIN_BLOB_AREA = 10

# 描边颜色
EDGE_COLOR = (255, 0, 0)

try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()

    sensor.set_framesize(Sensor.QVGA)       # 320x240
    sensor.set_pixformat(Sensor.RGB565)     # 彩色输入

    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=True)

    MediaManager.init()
    sensor.run()
    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()

        # 1. 图像获取与二值化
        raw_img = sensor.snapshot()
        display_img = raw_img.copy()
        display_img.binary([BLUE_THRESHOLD])

        # 2. 转灰度图 + 形态学平滑
        gray_img = display_img.to_grayscale(copy=True)
        try:
            gray_img.open(1)
        except:
            pass

        # ================= 核心修复：给灰度图四周加1像素黑边 =================
        img_w = gray_img.width()
        img_h = gray_img.height()
        # 上下边缘全部设为黑色
        for x in range(img_w):
            gray_img.set_pixel(x, 0, 0)
            gray_img.set_pixel(x, img_h - 1, 0)
        # 左右边缘全部设为黑色
        for y in range(img_h):
            gray_img.set_pixel(0, y, 0)
            gray_img.set_pixel(img_w - 1, y, 0)
        # 处理后边界像素有了完整邻域，边缘检测可正常识别

        # 找色块（灰度图上阈值格式正确）
        blobs = gray_img.find_blobs([(255,255)], pixels_threshold=MIN_BLOB_AREA, area_threshold=MIN_BLOB_AREA, merge=True)

        if blobs:
            blobs.sort(key=lambda b: b.area(), reverse=True)
            b = blobs[0]
            rx, ry, rw, rh = b.rect()

            # 3. 边缘检测
            gray_img.find_edges(image.EDGE_SIMPLE, threshold=(100, 255))

            # 4. 只在色块矩形范围内描绘边沿
            step = 1
            for y in range(ry, ry + rh, step):
                for x in range(rx, rx + rw, step):
                    if gray_img.get_pixel(x, y) > 200:
                        display_img.draw_circle(x, y, 2, color=EDGE_COLOR, fill=True)

        # 5. 缩放全屏显示
        try:
            scaled_img = display_img.copy(x_scale=2, y_scale=2)
            Display.show_image(scaled_img)
            scaled_img = None
        except Exception:
            Display.show_image(display_img)

        raw_img = None
        display_img = None
        gray_img = None
        gc.collect()
        print("FPS:", clock.fps())

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
