import time, os, sys, gc

from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================================================
#                                    彩色摄像头画面预览（CanMV IDE 可见）
# -----------------------------------------------------------------------------------------------------
# 用途：单独确认摄像头是否出图、方向对不对、端口接线是否正常。
# 在 CanMV IDE 里点"运行"后，右下角即可看到实时彩色画面。
# 本文件不做任何识别，只负责把原始彩色图送到屏幕和 IDE。
# =====================================================================================================

# ------------------------------ 可调参数 ------------------------------
FRAME_SIZE = Sensor.QVGA      # Sensor.QVGA(320x240) / Sensor.VGA(640x480)
TO_IDE     = True             # True = CanMV IDE 也能看到画面
SHOW_FPS   = True

# 显示缓冲尺寸（ST7701 屏幕为 640x480）
DISPLAY_W = 640
DISPLAY_H = 480

# ------------------------------ 图像缩放 ------------------------------
# 屏幕是 640x480，若取图用 QVGA(320x240)，显示前放大 2 倍刚好铺满。
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
    sensor.set_pixformat(Sensor.RGB565)      # 彩色输入

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
        img = sensor.snapshot()

        # ---------------- 叠加信息 ----------------
        if SHOW_FPS:
            try:
                img.draw_string(6, 6, "fps:%d %dx%d" % (fps, img.width(), img.height()),
                                color=(255, 255, 0), scale=1.5)
            except Exception:
                pass

        # ---------------- 显示到屏幕 / IDE ----------------
        try:
            scaled_img = img.copy(x_scale=SCALE_X, y_scale=SCALE_Y)
            Display.show_image(scaled_img)
            scaled_img = None
        except Exception:
            Display.show_image(img)

        now = time.ticks_ms()
        if now - fps_t >= 1000:
            fps = int(clock.fps())
            fps_t = now

        img = None
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
