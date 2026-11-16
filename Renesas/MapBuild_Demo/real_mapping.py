"""瑞萨板真实数据实时显示，串口线程只接收，Tk 主线程更新地图。"""
import math
import queue
import struct
import threading
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from matplotlib.patches import Rectangle
from matplotlib.collections import PatchCollection
import numpy as np
from simulation import Grid, defaults


def crc16(data):
    crc = 0xffff
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


def map_rays(rays, reverse):
    """Convert the board's ray angles for the selected PC-side convention."""
    return [(-angle if reverse else angle, distance, hit)
            for angle, distance, hit in rays]


def relative_pose(pose, origin):
    """Translate odometry into a fixed frame anchored at the first received pose."""
    return (pose[0] - origin[0], pose[1] - origin[1], pose[2])


def real_map_settings():
    settings = defaults()['map']
    settings.update(width=100, height=100, resolution=0.06, origin_x=-3.0, origin_y=-3.0)
    return settings


def obstacle_boxes(occupied, min_cells=1):
    """Axis-aligned boxes of 8-connected occupied cells, in grid coordinates."""
    h, w = occupied.shape
    visited = np.zeros((h, w), dtype=bool)
    boxes = []
    for y, x in np.argwhere(occupied):
        if visited[y, x]: continue
        visited[y, x] = True
        stack = [(int(y), int(x))]
        count = 0; min_x = max_x = int(x); min_y = max_y = int(y)
        while stack:
            cy, cx = stack.pop(); count += 1
            min_x = min(min_x, cx); max_x = max(max_x, cx)
            min_y = min(min_y, cy); max_y = max(max_y, cy)
            for ny in range(max(0, cy-1), min(h, cy+2)):
                for nx in range(max(0, cx-1), min(w, cx+2)):
                    if occupied[ny, nx] and not visited[ny, nx]:
                        visited[ny, nx] = True; stack.append((ny, nx))
        if count >= min_cells: boxes.append((min_x, min_y, max_x+1, max_y+1))
    return boxes


class StreamParser:
    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf.extend(data)
        result = []
        while len(self.buf) >= 4:
            if self.buf[:2] != b'\xa5\x5a':
                del self.buf[0]
                continue
            length = int.from_bytes(self.buf[2:4], 'little')
            if length != 2519 and (length < 17 or length > 145 or (length - 17) % 8):
                self.bad += 1; del self.buf[0]; continue
            total = length + 6
            if len(self.buf) < total: break
            frame = self.buf[:total]
            if crc16(frame[2:-2]) != int.from_bytes(frame[-2:], 'little'):
                self.bad += 1; del self.buf[0]; continue
            if length == 2519:
                if frame[4:6] != b'M\x03' or frame[22] > 1:
                    self.bad += 1; del self.buf[:total]; continue
                seq, x, y, yaw = struct.unpack_from('<Ifff', frame, 6)
                if not all(math.isfinite(v) for v in (x, y, yaw)):
                    self.bad += 1; del self.buf[:total]; continue
                packed = np.frombuffer(frame, dtype=np.uint8, count=2500, offset=23)
                cells = ((packed[:, None] >> np.array([0, 2, 4, 6], dtype=np.uint8)) & 3).reshape(100, 100)
                result.append(('map', seq, (x, y, yaw) if frame[22] else None, cells))
                del self.buf[:total]; continue
            seq, x, y, yaw, count = struct.unpack_from('<IfffB', frame, 4)
            if count > 16 or length != 17 + 8 * count:
                self.bad += 1; del self.buf[0]; continue
            rays = [(*struct.unpack_from('<ff', frame, 21 + 8*i), 1) for i in range(count)]
            if not all(math.isfinite(v) for v in (x, y, yaw)) or any(not math.isfinite(a) or not math.isfinite(d) or d <= 0 for a, d, _ in rays):
                self.bad += 1; del self.buf[:total]; continue
            result.append((seq, (x, y, yaw), rays))
            del self.buf[:total]
        return result


class RealMappingWindow:
    def __init__(self, parent, library):
        self.win = tk.Toplevel(parent); self.win.title('真实数据实时建图'); self.win.geometry('850x800')
        self.win.protocol('WM_DELETE_WINDOW', self.close)
        self.lib = Path(library).with_name('libmapreal.dll' if Path(library).suffix == '.dll' else 'libmapreal.so')
        self.grid = None; self.device = None; self.worker = None; self.stop = threading.Event()
        self.queue = queue.Queue(maxsize=2000); self.parser = StreamParser(); self.last = None; self.gaps = 0; self.count = 0; self.synced = False
        top = ttk.Frame(self.win); top.pack(fill='x', padx=8, pady=6)
        ttk.Label(top, text='串口').pack(side='left'); self.port = tk.StringVar()
        self.ports = ttk.Combobox(top, textvariable=self.port, width=12); self.ports.pack(side='left', padx=4)
        ttk.Button(top, text='刷新', command=self.refresh).pack(side='left')
        ttk.Button(top, text='连接', command=self.connect).pack(side='left', padx=4)
        ttk.Button(top, text='断开', command=self.disconnect).pack(side='left')
        ttk.Label(top, text='显示板端地图快照').pack(side='left', padx=8)
        serial_options = ttk.Frame(self.win); serial_options.pack(fill='x', padx=8)
        ttk.Label(serial_options, text='波特率').pack(side='left')
        self.baud = tk.StringVar(value='9600')
        self.baud_select = ttk.Combobox(serial_options, textvariable=self.baud, width=12,
                                        values=('9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600'))
        self.baud_select.pack(side='left', padx=4)
        ttk.Label(serial_options, text='8N1（可输入其他波特率；连接后修改需重新连接）').pack(side='left')
        self.status = tk.StringVar(value='连接后等待板端地图快照，范围 -3 至 +3 米。')
        ttk.Label(self.win, textvariable=self.status).pack(fill='x', padx=8)
        self.fig = Figure(figsize=(7, 7)); self.fig.subplots_adjust(left=.12, right=.95, bottom=.12, top=.95); self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, self.win); self.canvas.get_tk_widget().pack(fill='both', expand=True)
        self.refresh(); self.clear(); self.win.after(100, self.tick)

    def refresh(self):
        try:
            from serial.tools import list_ports
            ports = [p.device for p in list_ports.comports()]
            self.ports['values'] = ports
            if not self.port.get() and ports: self.port.set(ports[0])
        except ImportError:
            self.status.set('缺少 pyserial：请运行 python -m pip install pyserial')

    def clear(self):
        if self.grid: self.grid.close()
        self.grid = None; self.cells = np.zeros((100, 100), dtype=np.uint8)
        self.count = 0; self.last = None; self.gaps = 0; self.synced = False
        self.draw(None)

    def angle_changed(self, _event=None):
        self.clear()
        self.status.set('角度方向已更改并清空电脑地图；电脑端反向时与板端地图不同。')

    def connect(self):
        if self.device: return
        try:
            baud = int(self.baud.get().strip())
            if baud <= 0:
                raise ValueError('波特率必须是大于 0 的整数')
        except ValueError:
            messagebox.showerror('波特率错误', '请输入大于 0 的整数波特率。', parent=self.win); return
        try:
            import serial
            self.device = serial.Serial(self.port.get(), baud, timeout=0.1)
        except Exception as exc:
            messagebox.showerror('串口连接失败', str(exc), parent=self.win); return
        self.stop.clear(); self.parser = StreamParser(); self.queue = queue.Queue(maxsize=2000); self.clear()
        self.worker = threading.Thread(target=self.read_loop, daemon=True); self.worker.start()
        self.status.set(f'已连接 {self.port.get()}，等待板端地图快照；请确认烧录了支持快照的固件。')

    def read_loop(self):
        try:
            while not self.stop.is_set():
                data = self.device.read(1024)
                for frame in self.parser.feed(data):
                    if frame[0] != 'map': continue
                    try: self.queue.put_nowait(frame)
                    except queue.Full:
                        self.queue.get_nowait(); self.queue.put_nowait(('error', '电脑处理队列已满，地图可能不一致')); return
        except Exception as exc:
            if not self.stop.is_set(): self.queue.put(('error', str(exc)))

    def tick(self):
        if not self.win.winfo_exists(): return
        pose = None; received = False
        for _ in range(500):
            try: item = self.queue.get_nowait()
            except queue.Empty: break
            if item[0] == 'error': self.status.set('串口异常：' + item[1]); self.disconnect(); break
            _, seq, pose, self.cells = item
            self.last = seq; self.count += 1; received = True
        if received:
            self.draw(pose)
            self.status.set(f'显示板端闭运算地图；快照 {self.count}，板端帧 {self.last}，CRC/格式错误 {self.parser.bad}')
        self.win.after(100, self.tick)

    def draw(self, pose):
        cfg = real_map_settings(); cells = self.cells
        values = np.where(cells >= 2, 0.0, np.where(cells == 1, 1.0, 0.5))
        extent = (cfg['origin_x'], cfg['origin_x'] + cfg['width'] * cfg['resolution'],
                  cfg['origin_y'], cfg['origin_y'] + cfg['height'] * cfg['resolution'])
        self.ax.clear(); self.ax.imshow(values, origin='lower', extent=extent, cmap='gray', vmin=0, vmax=1)
        boxes = [Rectangle((extent[0] + x0 * cfg['resolution'], extent[2] + y0 * cfg['resolution']),
                           (x1-x0) * cfg['resolution'], (y1-y0) * cfg['resolution'])
                 for x0, y0, x1, y1 in obstacle_boxes(cells == 2)]
        if boxes: self.ax.add_collection(PatchCollection(boxes, facecolor='none', edgecolor='#ff9e00', linewidth=1.0))
        if pose: self.ax.plot(pose[0], pose[1], 'ro', markersize=4); self.ax.arrow(pose[0],pose[1],0.2*math.cos(pose[2]),0.2*math.sin(pose[2]),color='red',head_width=.06)
        self.ax.set_xlim(extent[:2]); self.ax.set_ylim(extent[2:])
        self.ax.set_xlabel('X (m)'); self.ax.set_ylabel('Y (m)'); self.ax.set_aspect('equal'); self.canvas.draw_idle()

    def disconnect(self):
        self.stop.set()
        if self.device:
            self.device.close()
            if self.worker: self.worker.join(timeout=1)
            self.device = None

    def close(self):
        self.disconnect()
        if self.grid: self.grid.close()
        self.win.destroy()
