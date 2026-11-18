import time, os, sys, gc, math
import image
from media.sensor import *
from media.display import *
from media.media import *
from machine import UART

#       遇到的问题                                  调整方向
#不规则物体、带尖刺的蓝色块还被选中	  调大 MIN_COMPACTNESS（比如到 0.62），提高平整门槛
#A4 纸、规则矩形被漏掉了             调小 MIN_COMPACTNESS（比如到 0.48），放宽形状要求
#边缘太粗 / 太细                   修改 EDGE_RADIUS，数值越大线越粗
#边缘断点多、不连贯                  调小 EDGE_LOW_THRESH，增加边缘点数量
#噪点多、边缘杂乱                   调大 MORPH_OPEN_KERNEL，加强去毛刺；或调大 EDGE_LOW_THRESH
#小目标都被过滤了                   调小 MIN_BLOB_AREA
#想换描边颜色                     修改 EDGE_COLOR 的 RGB 值，比如蓝色 (0,0,255)

# ==============================================================
# ******************** 所有可调参数（统一在这里改） ********************
# ==============================================================

# -------------------- 1. 颜色阈值 --------------------
# LAB色彩空间蓝色阈值，控制哪些颜色会被识别为目标
# 格式：[L_min, L_max, A_min, A_max, B_min, B_max]
BLUE_THRESHOLD = [0, 100, -3, 127, -128, -13]

# -------------------- 2. 图像预处理 --------------------
MORPH_OPEN_KERNEL = 1          # 形态学开运算核大小，数值越大去毛刺越强，目标边缘越平滑
EDGE_LOW_THRESH = 100          # 边缘检测低阈值，越小边缘越多、越细密
EDGE_HIGH_THRESH = 255         # 边缘检测高阈值，越大边缘越少、越干净
EDGE_PIXEL_THRESH = 200        # 边缘点灰度判定阈值，大于此值视为边缘
BORDER_PAD_SIZE = 1            # 四周黑边宽度，解决目标贴屏幕边缘时断边问题

# -------------------- 3. 基础筛选 --------------------
MIN_BLOB_AREA = 100            # 最小色块面积（像素），小于的直接当噪点过滤
MIN_CONTOUR_POINTS = 20        # 最少轮廓点数，小于的直接跳过
MAX_CONTOUR_POINTS = 2000      # 最大轮廓点数，防止死循环

# -------------------- 4. 形状筛选（核心） --------------------
MIN_COMPACTNESS = 0.55         # ★ 最小紧凑度（平整程度）
                               # 越接近1要求越严格，只保留非常光滑的形状
                               # 0.45 很宽松 | 0.55 适中 | 0.65 严格
                               # 有尖锐凹凸、毛刺多的物体会低于此值被排除

# -------------------- 5. 描边显示 --------------------
EDGE_COLOR = (255, 0, 0)       # 描边颜色 (R, G, B)，默认红色
EDGE_RADIUS = 2                # 描边圆点半径，控制线条粗细，数值越大线越粗
ENABLE_SCALE = True            # 是否启用缩放显示 True/False
SCALE_FACTOR = 2               # 缩放倍数，默认放大2倍全屏显示

# -------------------- 6. 坐标与中心标记 --------------------
SHOW_SCREEN_X_AXIS = True      # 是否显示屏幕中心X轴（水平线，仅作参考）
AXIS_COLOR = (0, 255, 0)       # 坐标轴颜色，默认绿色
AXIS_THICKNESS = 1             # 坐标轴线条粗细

SHOW_ORIGIN = True             # 是否显示坐标原点（屏幕中心，X轴零点）
ORIGIN_COLOR = (0, 255, 0)     # 原点颜色，默认绿色
ORIGIN_SIZE = 6                # 原点十字大小

SHOW_TARGET_CENTER = True      # 是否显示目标中心点（画在目标真实中心）
TARGET_CENTER_COLOR = (0, 0, 255) # 目标中心颜色，默认蓝色
TARGET_CENTER_SIZE = 8         # 目标中心十字大小

# -------------------- 7. 硬件串口输出 --------------------
# UART1 对应引脚：TX=GPIO40，RX=GPIO41（K230硬件固定映射，无需额外指定）
UART_NUM = 1                   # 串口号：UART1
UART_BAUDRATE = 115200         # 波特率
UART_END = "\r\n"              # 数据结尾换行符，方便接收端解析

# ==============================================================
# ******************** 以下为算法逻辑，一般不用改 ********************
# ==============================================================

# ---------------------- 辅助几何函数 ----------------------
def point_dist(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)

# 边界跟踪：从边缘图中提取有序轮廓点
def trace_contour(edge_img, rx, ry, rw, rh):
    # 找起点：ROI内最靠上的边缘点
    start = None
    for y in range(ry, ry + rh):
        for x in range(rx, rx + rw):
            if edge_img.get_pixel(x, y) > EDGE_PIXEL_THRESH:
                start = (x, y)
                break
        if start:
            break
    if not start:
        return []

    # 8邻域顺时针方向
    dirs = [(1,0),(1,1),(0,1),(-1,1),(-1,0),(-1,-1),(0,-1),(1,-1)]
    contour = [start]
    current = start
    prev_dir = 2

    while True:
        start_dir = (prev_dir - 2) % 8
        found = False
        for i in range(8):
            d = (start_dir + i) % 8
            dx, dy = dirs[d]
            nx, ny = current[0]+dx, current[1]+dy
            if rx <= nx < rx+rw and ry <= ny < ry+rh:
                if edge_img.get_pixel(nx, ny) > EDGE_PIXEL_THRESH:
                    next_p = (nx, ny)
                    prev_dir = d
                    found = True
                    break
        if not found or next_p == start:
            break
        if next_p != contour[-1]:
            contour.append(next_p)
        current = next_p
        if len(contour) > MAX_CONTOUR_POINTS:
            break
    return contour

# 计算轮廓总周长
def calc_perimeter(contour):
    perimeter = 0
    n = len(contour)
    for i in range(n):
        perimeter += point_dist(contour[i], contour[(i+1)%n])
    return perimeter

# ---------------------- 主程序 ----------------------
uart = None
sensor = None
try:
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)       # 320x240
    sensor.set_pixformat(Sensor.RGB565)     # 彩色输入

    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=True)
    MediaManager.init()
    sensor.run()
    clock = time.clock()

    # 初始化硬件串口（K230 UART1 固定对应 TX=GPIO40, RX=GPIO41）
    try:
        uart = UART(UART_NUM, UART_BAUDRATE)
        print("UART1 init success, TX=GPIO40, RX=GPIO41")
    except Exception as uart_err:
        print(f"UART init failed: {uart_err}")
        uart = None

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
            gray_img.open(MORPH_OPEN_KERNEL)
        except:
            pass

        # 四周加黑边，解决贴边断边
        img_w = gray_img.width()
        img_h = gray_img.height()
        img_center_x = img_w // 2  # 屏幕水平中心（X轴原点）
        img_center_y = img_h // 2  # 屏幕垂直中心

        for x in range(img_w):
            for dy in range(BORDER_PAD_SIZE):
                gray_img.set_pixel(x, dy, 0)
                gray_img.set_pixel(x, img_h - 1 - dy, 0)
        for y in range(img_h):
            for dx in range(BORDER_PAD_SIZE):
                gray_img.set_pixel(dx, y, 0)
                gray_img.set_pixel(img_w - 1 - dx, y, 0)

        # 绘制屏幕中心X轴参考线
        if SHOW_SCREEN_X_AXIS:
            display_img.draw_line(0, img_center_y, img_w, img_center_y,
                                  color=AXIS_COLOR, thickness=AXIS_THICKNESS)

        # 绘制坐标原点（屏幕正中心，X轴零点）
        if SHOW_ORIGIN:
            display_img.draw_cross(img_center_x, img_center_y,
                                   color=ORIGIN_COLOR,
                                   size=ORIGIN_SIZE,
                                   thickness=2)

        # 3. 找所有独立色块（不合并，逐个判断形状）
        blobs = gray_img.find_blobs([(255,255)],
                                    pixels_threshold=MIN_BLOB_AREA,
                                    area_threshold=MIN_BLOB_AREA,
                                    merge=False)

        target_x = None
        target_area = None
        best_contour = None
        target_cx = None
        target_cy = None

        if blobs:
            # 一次全图边缘检测
            gray_img.find_edges(image.EDGE_SIMPLE, threshold=(EDGE_LOW_THRESH, EDGE_HIGH_THRESH))

            candidates = []
            # 逐个判断形状平整程度
            for b in blobs:
                rx, ry, rw, rh = b.rect()
                blob_cx = rx + rw // 2  # 目标真实中心X
                blob_cy = ry + rh // 2  # 目标真实中心Y
                blob_area = b.area()    # 目标像素面积

                # 提取该色块的边缘轮廓
                contour = trace_contour(gray_img, rx, ry, rw, rh)
                if len(contour) < MIN_CONTOUR_POINTS:
                    continue

                perimeter = calc_perimeter(contour)
                if perimeter == 0:
                    continue

                # 紧凑度公式：4π * 面积 / 周长²
                compactness = (4 * math.pi * blob_area) / (perimeter * perimeter)

                # 形状达标就加入候选，同时保存真实中心坐标
                if compactness >= MIN_COMPACTNESS:
                    candidates.append( (blob_area, contour, blob_cx, blob_cy) )

            # 从合格目标中选面积最大的一个
            if candidates:
                candidates.sort(key=lambda x: x[0], reverse=True)
                target_area, best_contour, target_cx, target_cy = candidates[0]
                # 计算以屏幕中心为原点的X轴偏移（左负右正）
                target_x = target_cx - img_center_x

                # 描边
                for x, y in best_contour:
                    display_img.draw_circle(x, y, EDGE_RADIUS, color=EDGE_COLOR, fill=True)

                # 绘制目标中心点（画在目标真实中心位置）
                if SHOW_TARGET_CENTER:
                    display_img.draw_cross(target_cx, target_cy,
                                           color=TARGET_CENTER_COLOR,
                                           size=TARGET_CENTER_SIZE,
                                           thickness=2)

        # 4. 串口输出：仅发X坐标和面积，不发Y
        if target_x is not None:
            output_str = f"X:{target_x} AREA:{target_area}"
            print(output_str + f" FPS:{clock.fps():.1f}")  # IDE调试带帧率
            if uart:
                uart.write(output_str + UART_END)        # 硬件串口仅发有效数据
        else:
            print(f"NO_TARGET FPS:{clock.fps():.1f}")
            if uart:
                uart.write("NO_TARGET" + UART_END)

        # 5. 显示画面
        if ENABLE_SCALE:
            try:
                scaled_img = display_img.copy(x_scale=SCALE_FACTOR, y_scale=SCALE_FACTOR)
                Display.show_image(scaled_img)
                scaled_img = None
            except Exception:
                Display.show_image(display_img)
        else:
            Display.show_image(display_img)

        raw_img = None
        display_img = None
        gray_img = None
        gc.collect()

except KeyboardInterrupt as e:
    print("user stop: ", e)
except BaseException as e:
    print(f"Exception {e}")
finally:
    if uart is not None:
        try:
            uart.deinit()
        except:
            pass
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()
