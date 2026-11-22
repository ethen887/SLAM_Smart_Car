import time, os, sys, gc

from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================================================
#                                  二值化画面预览（蓝色 -> 白色，其他 -> 黑色）
# -----------------------------------------------------------------------------------------------------
# 用途：调 BLUE_THRESHOLD 时用。屏幕上只剩一个白色区域，就是被识别成"蓝色"的部分，
#       白色区域越大越干净，说明阈值越合适；有大量白色噪点则说明阈值太宽。
# 在 CanMV IDE 里点"运行"后，右下角即可看到实时二值图。
# =====================================================================================================

# ------------------------------ 颜色阈值 (LAB) ------------------------------
# 与 blue_auto.py 里的 BLUE_THRESHOLD 保持同一套，调这里就等于调识别阈值
BLUE_THRESHOLD = [0, 100, -128, 127, -128, -24]

# ------------------------------ 可调参数 ------------------------------
FRAME_SIZE = Sensor.QVGA      # Sensor.QVGA(320x240) / Sensor.VGA(640x480)
TO_IDE     = True             # True = CanMV IDE 也能看到画面
SHOW_FPS   = True

# 显示缓冲尺寸（ST7701 屏幕为 640x480）
DISPLAY_W = 640
DISPLAY_H = 480

# ------------------------------ 图像缩放 ------------------------------
if FRAME_SIZE == Sensor.QVGA:
    SCALE_X = 2
    SCALE_Y = 2
else:
    SCALE_X = 1
    SCALE_Y = 1

# =====================================================================================================

sensor = None

try:
    # ---------------- 摄像头初始化 ----------------
    sensor = Sensor(width=1280, height=960)
    sensor.reset()

    sensor.set_hmirror(False)
    sensor.set_vflip(False)

    sensor.set_framesize(FRAME_SIZE)
    sensor.set_pixformat(Sensor.RGB565)      # 彩色输入，二值化在帧缓冲里做

    # ---------------- 显示屏 + IDE 预览 ----------------
    Display.init(Display.ST7701, width=DISPLAY_W, height=DISPLAY_H, fps=60, to_ide=TO_IDE)
    MediaManager.init()
    sensor.run()

    clock = time.clock()
    fps = 0
    fps_t = time.ticks_ms()

    while True:
        os.exitpoint()
        clock.tick()

        # ---------------- 取一帧彩色图 ----------------
        raw_img = sensor.snapshot()

        # ---------------- 二值化：落在阈值内的变白，其余变黑 ----------------
        # binary() 直接把图像就地改成黑白两色，不再需要手动逐像素判断
        bin_img = raw_img.copy()
        bin_img.binary([BLUE_THRESHOLD])

        # ---------------- 叠加信息 ----------------
        if SHOW_FPS:
            try:
                bin_img.draw_string(6, 6, "fps:%d" % fps,
                                    color=(255, 255, 255), scale=1.5)
            except Exception:
                pass

        # ---------------- 显示到屏幕 / IDE ----------------
        try:
            scaled_img = bin_img.copy(x_scale=SCALE_X, y_scale=SCALE_Y)
            Display.show_image(scaled_img)
            scaled_img = None
        except Exception:
            Display.show_image(bin_img)

        now = time.ticks_ms()
        if now - fps_t >= 1000:
            fps = int(clock.fps())
            fps_t = now

        raw_img = None
        bin_img = None
        gc.collect()

        time.sleep_ms(10)

except KeyboardInterrupt:
    pass
except BaseException as e:
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
