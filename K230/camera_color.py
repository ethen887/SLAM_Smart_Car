import time, os, gc

from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================================================
#                              彩色画面预览（板载屏幕 + CanMV IDE）
# -----------------------------------------------------------------------------------------------------
# 用途：确认摄像头取景范围、光照方向、A4 纸在画面里的位置和大小。
#       画面原样显示，不做任何颜色处理，用来和 camera_binary.py 对照。
# 操作：CanMV IDE 里直接运行，右侧 Frame Buffer 能看到实时画面。
# =====================================================================================================

TO_IDE = True       # True = 同时显示到 CanMV IDE

try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(Sensor.QVGA)       # 320x240，与 blue_auto.py 一致
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=TO_IDE)
    MediaManager.init()
    sensor.run()

    clock = time.clock()

    while True:
        os.exitpoint()
        clock.tick()

        img = sensor.snapshot()

        # 左上角叠一行帧率，方便判断画面是否卡顿
        try:
            img.draw_string(4, 4, "FPS %.1f" % clock.fps(),
                            color=(255, 255, 0), scale=2)
        except Exception:
            pass

        # 320x240 放大 2 倍显示，占满 640x480 屏幕
        try:
            scaled = img.copy(x_scale=2, y_scale=2)
            Display.show_image(scaled)
            scaled = None
        except Exception:
            Display.show_image(img)

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