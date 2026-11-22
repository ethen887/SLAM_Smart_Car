import time, os, gc, struct

from media.sensor import *
from media.display import *
from media.media import *
from machine import UART, FPIOA

# =====================================================================================================
#                              蓝色目标识别 + 边界描绘 + 中心坐标发送
# -----------------------------------------------------------------------------------------------------
# 流程：彩色取图 -> 二值化(蓝色变白) -> 转灰度 -> 形态学平滑 -> 加黑边 ->
#       找最大白块 -> 描出它的白色边界 -> 用边界包围盒算中心 -> 发串口。
# 坐标原点：屏幕中心。向左为负，向右为正。
# =====================================================================================================

# ------------------------------ 1. 颜色阈值 (LAB) ------------------------------
# L 上限从 100 放开到 255：蓝色 A4 纸在灯光下亮度经常超过 100，
# 原来那版会把纸的亮部当"不是蓝色"整片排除，只剩边缘一条斜带变白，
# 于是矩形度 f 只有 0.4x，跟干扰斜条一个水平，怎么调 FILL_MIN 都分不开。
# 真正的"必须是蓝的"由 b 上限 -24 卡住（白墙/灯光 b≈0，进不来），所以放开 L 是安全的。
BLUE_THRESHOLD = [0, 255, -128, 127, -128, -24]

# ------------------------------ 2. 目标筛选 ------------------------------
MIN_BLOB_AREA = 800          # 白色区域小于该像素数就丢掉，排除小噪点
# 宽高比筛选：A4 纸竖放，正常时 高/宽 ≈ 1.41（29.7/21）。
# 车贴近后纸的下半部分移出画面，比例会急剧变小，所以下限放到 0.20。
ASPECT_MIN    = 0.50          # 高/宽 下限：低于它是横条（地面反光、桌沿）
ASPECT_MAX    = 2.50          # 高/宽 上限：高于它是竖条（门框、柱子、线缆）
# 扁条特判（关键）：近距离的 A4 纸虽然高/宽很小，但它是“左右撑满画面”的；
# 而干扰性的斜条、地面反光只占画面中间一小段。所以扁条必须够宽才算目标。
WIDE_RATIO    = 0.60          # 高/宽 小于该值视为“扁条”
WIDE_MIN_FRAC = 0.65          # 扁条的宽度至少要占画面宽度的该比例
# 矩形度筛选：白色像素数 / 外接框面积。A4 纸二值化后边缘有锯齿，
# 实测往往只有 0.45~0.65，所以不能设太高，否则真纸反而被筛掉。
FILL_MIN      = 0.5         # 矩形度下限
MIN_BLOB_SIDE = 30           # 外接框任一边小于该像素数就不算目标

# 调参辅助：True = 屏幕上画出每个候选的外接框、尺寸、比例、矩形度，
# 绿色表示通过筛选，红色表示被筛掉。调完参数改成 False 即可关掉。
DEBUG_DRAW    = True

# ------------------------------ 3. 描边与显示 ------------------------------
EDGE_COLOR = (255, 0, 0)      # 边界点颜色（红）
DRAW_STEP  = 2                # 每 2 个边界点画一个圆，减少绘制开销
TO_IDE     = True            # CanMV IDE 里也能看到画面

# ------------------------------ 4. 串口参数 ------------------------------
UART_TX_PIN = 40              # K230 TXD  -> RA8P1 P715 (RXD4)
UART_RX_PIN = 41              # K230 RXD  <- RA8P1 P714 (TXD4)
UART_BAUD   = 115200

# ------------------------------ 5. 坐标系 ------------------------------
IMG_W       = 320             # QVGA 宽
IMG_H       = 240             # QVGA 高
CENTER_X    = IMG_W // 2      # 160，坐标原点
CENTER_Y    = IMG_H // 2      # 120
COORD_SCALE = 2               # RA8P1 侧按 640 宽计算，这里换算过去
AREA_SCALE  = 4               # 面积同样按 4 倍换算

# =====================================================================================================
#                                  串口协议（与 RA8P1 侧一致）
# -----------------------------------------------------------------------------------------------------
# 视觉帧：AA 55 02 10 00 | x(int16) | y(int16) | box_w(uint16) | box_h(uint16) | area(u32) | ts(u32)
#         | CRC | 0D 0A（共 25 字节）
# CRC：CRC-16/MODBUS，初值 0xFFFF，多项式 0xA001，校验范围 frame[2:21]
# =====================================================================================================

FRAME_HEAD0       = 0xAA
FRAME_HEAD1       = 0x55
FRAME_TYPE_VISION = 0x02
FRAME_LEN_VISION  = 0x10
FRAME_SIZE_VISION = 25


def crc16_modbus(data):
    """CRC-16/MODBUS，与 RA8P1 侧 link_k230.c 完全一致。"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def build_vision_frame(offset_x, offset_y, box_w, box_h, area, timestamp):
    """打包一帧视觉数据（25 字节）。"""
    frame = bytearray(FRAME_SIZE_VISION)
    frame[0] = FRAME_HEAD0
    frame[1] = FRAME_HEAD1
    frame[2] = FRAME_TYPE_VISION
    frame[3] = FRAME_LEN_VISION
    frame[4] = 0x00
    struct.pack_into("<hhHHII", frame, 5,
                     offset_x, offset_y, box_w, box_h, area, timestamp)

    crc = crc16_modbus(frame[2:21])
    frame[21] = crc & 0xFF
    frame[22] = (crc >> 8) & 0xFF
    frame[23] = 0x0D
    frame[24] = 0x0A
    return frame


def extract_edge_points(gray_img, rect):
    """
    在色块外接框内提取白色边界点，返回 (点列表, 包围盒)。
    find_edges 之后，亮像素就是边界。
    """
    rx, ry, rw, rh = rect

    x0 = max(0, rx)
    y0 = max(0, ry)
    x1 = min(IMG_W - 1, rx + rw - 1)
    y1 = min(IMG_H - 1, ry + rh - 1)
    if x1 < x0 or y1 < y0:
        return None, None

    pts = []
    min_x, max_x = IMG_W, -1
    min_y, max_y = IMG_H, -1

    for y in range(y0, y1 + 1, 2):
        for x in range(x0, x1 + 1, 2):
            if gray_img.get_pixel(x, y) > 200:
                pts.append((x, y))
                if x < min_x:
                    min_x = x
                if x > max_x:
                    max_x = x
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y

    if not pts:
        return None, None

    return pts, (min_x, min_y, max_x - min_x + 1, max_y - min_y + 1)


# ========================= 主程序 =========================

sensor = None
uart = None

try:
    # ---------------- 串口 ----------------
    fpioa = FPIOA()
    fpioa.set_function(UART_TX_PIN, FPIOA.UART1_TXD)
    fpioa.set_function(UART_RX_PIN, FPIOA.UART1_RXD)
    uart = UART(UART.UART1, baudrate=UART_BAUD, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    # ---------------- 摄像头 ----------------
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_hmirror(False)
    sensor.set_vflip(False)
    sensor.set_framesize(Sensor.QVGA)       # 320x240
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701, width=640, height=480, fps=60, to_ide=TO_IDE)
    MediaManager.init()
    sensor.run()

    while True:
        os.exitpoint()

        # ---------------- 1. 取图 + 二值化 ----------------
        display_img = sensor.snapshot()
        display_img.binary([BLUE_THRESHOLD])     # 阈值内变白，其余变黑

        # ---------------- 2. 转灰度 + 形态学平滑 ----------------
        gray_img = display_img.to_grayscale(copy=True)
        try:
            gray_img.open(1)
        except Exception:
            pass

        # ---------------- 3. 四周补 1 像素黑边（find_edges 需要完整邻域） ----------------
        img_w = gray_img.width()
        img_h = gray_img.height()
        for x in range(img_w):
            gray_img.set_pixel(x, 0, 0)
            gray_img.set_pixel(x, img_h - 1, 0)
        for y in range(img_h):
            gray_img.set_pixel(0, y, 0)
            gray_img.set_pixel(img_w - 1, y, 0)

        # ---------------- 4. 先画出坐标原点（每帧都显示，与目标是否存在无关） ----------------
        try:
            display_img.draw_cross(CENTER_X, CENTER_Y, color=(0, 255, 0), size=12, thickness=2)
            display_img.draw_circle(CENTER_X, CENTER_Y, 12, color=(0, 255, 0), thickness=2)
        except Exception:
            pass

        # ---------------- 5. 找白色区域，取面积最大的一个 ----------------
        blobs = gray_img.find_blobs([(255, 255)],
                                    pixels_threshold=MIN_BLOB_AREA,
                                    area_threshold=MIN_BLOB_AREA,
                                    merge=True)

        target = None
        candidates = []
        debug_items = []
        if blobs:
            # 逐条判据依次检查，任一不满足就丢掉该候选；同时把打分信息记下来，
            # 方便 DEBUG_DRAW 打开时在屏幕上看到每个候选为什么被筛。
            for b in blobs:
                bw = b.w()
                bh = b.h()
                ratio = (float(bh) / float(bw)) if bw > 0 else 0.0
                fill = (float(b.area()) / float(bw * bh)) if (bw > 0 and bh > 0) else 0.0

                ok = True
                if bw < MIN_BLOB_SIDE or bh < MIN_BLOB_SIDE:
                    ok = False
                elif not (ASPECT_MIN <= ratio <= ASPECT_MAX):
                    ok = False
                elif (ratio < WIDE_RATIO) and (bw < WIDE_MIN_FRAC * IMG_W):
                    ok = False
                elif fill < FILL_MIN:
                    ok = False

                if ok:
                    candidates.append(b)
                if DEBUG_DRAW:
                    debug_items.append((b, bw, bh, ratio, fill, ok))

            if candidates:
                target = candidates[0]
                for b in candidates:
                    if b.area() > target.area():
                        target = b
            if candidates:
                target = candidates[0]
                for b in candidates:
                    if b.area() > target.area():
                        target = b

        # ---------------- 5.5 调参辅助：画出每个候选的框和实测参数 ----------------
        # 绿框 = 通过全部筛选项，红框 = 被筛掉；
        # 框上文字 = 宽x高  高宽比r  矩形度f；顶部 = 候选总数/合格数 + 当前阈值。
        if DEBUG_DRAW:
            try:
                for (b, bw, bh, ratio, fill, ok) in debug_items:
                    rx, ry, rw, rh = b.rect()
                    color = (0, 255, 0) if ok else (255, 0, 0)
                    display_img.draw_rectangle(rx, ry, rw, rh, color=color, thickness=1)
                    display_img.draw_string(rx + 2, ry + 2,
                                            "%dx%d r%.2f f%.2f" % (bw, bh, ratio, fill),
                                            color=color, scale=1)
                display_img.draw_string(
                    2, 2,
                    "n%d ok%d A%.2f-%.2f F%.2f W%.2f/%.2f" % (
                        len(debug_items), len(candidates),
                        ASPECT_MIN, ASPECT_MAX, FILL_MIN,
                        WIDE_RATIO, WIDE_MIN_FRAC),
                    color=(255, 255, 255), scale=1)
            except Exception:
                pass

        if target is not None:
            # ---------------- 6. 描边界 ----------------
            gray_img.find_edges(image.EDGE_SIMPLE, threshold=(100, 255))
            pts, box = extract_edge_points(gray_img, target.rect())

            if pts is not None:
                try:
                    for (px, py) in pts[::DRAW_STEP]:
                        display_img.draw_circle(px, py, 2, color=EDGE_COLOR, fill=True)
                except Exception:
                    pass

                # ---------------- 7. 边界包围盒中心 -> 以屏幕中心为原点的坐标 ----------------
                ex, ey, ew, eh = box
                center_x = ex + ew // 2
                center_y = ey + eh // 2

                offset_x = (center_x - CENTER_X) * COORD_SCALE
                offset_y = (center_y - CENTER_Y) * COORD_SCALE
                box_w    = ew * COORD_SCALE
                box_h    = eh * COORD_SCALE
                area     = target.area() * AREA_SCALE

                try:
                    # 原点到蓝色中心的连线，直观看出偏移方向
                    display_img.draw_line(CENTER_X, CENTER_Y, center_x, center_y,
                                          color=(0, 255, 255), thickness=2)
                    # 蓝色目标中心
                    display_img.draw_cross(center_x, center_y, color=(255, 255, 0), size=10, thickness=2)
                except Exception:
                    pass

                # ---------------- 8. 发送 ----------------
                uart.write(build_vision_frame(offset_x, offset_y, box_w, box_h, area,
                                              time.ticks_ms() & 0xFFFFFFFF))

        # ---------------- 9. 显示 ----------------
        try:
            scaled_img = display_img.copy(x_scale=2, y_scale=2)
            Display.show_image(scaled_img)
            scaled_img = None
        except Exception:
            Display.show_image(display_img)

        display_img = None
        gray_img = None
        gc.collect()

        time.sleep_ms(15)

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
