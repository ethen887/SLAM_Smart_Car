import time, os, gc

from media.sensor import *
from media.display import *
from media.media import *

# =====================================================================================================
#                              二值化预览（蓝色变白、其余变黑）
# -----------------------------------------------------------------------------------------------------
# 用途：调 BLUE_THRESHOLD 用。屏幕上一眼就能看出阈值是不是把蓝色 A4 纸
#       完整地变成白色块、把背景（白墙、地面、灯光）都压成黑的。
# 操作：CanMV IDE 里运行，改下面 BLUE_THRESHOLD 的六个数字后重新运行即可。
#       BLUE_THRESHOLD 的格式是 [L_MIN, L_MAX, A_MIN, A_MAX, B_MIN, B_MAX]。
# =====================================================================================================

# 与 blue_auto.py 完全一致的颜色阈值，改这里要同步改那边
BLUE_THRESHOLD = [0, 255, -128, 127, -128, -24]

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

        # 阈值内的像素变白，其余变黑。注意这一步是原地修改。
        img.binary([BLUE_THRESHOLD])

        # 二值化后图像已经是灰度格式，彩色文字可能抛异常，所以包一层
        try:
            img.draw_string(4, 4, "FPS %.1f" % clock.fps(),
                            color=(255, 255, 0), scale=2)
        except Exception:
            pass

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