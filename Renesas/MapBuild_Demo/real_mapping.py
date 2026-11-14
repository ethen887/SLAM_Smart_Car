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
import numpy as np
from simulation import Grid, defaults


def crc16(data):
    crc = 0xffff
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ (0xa001 if crc & 1 else 0)
    return crc


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
            if length < 17 or length > 145 or (length - 17) % 8:
                self.bad += 1; del self.buf[0]; continue
            total = length + 6
            if len(self.buf) < total: break
            frame = self.buf[:total]
            if crc16(frame[2:-2]) != int.from_bytes(frame[-2:], 'little'):
                self.bad += 1; del self.buf[0]; continue
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
        ttk.Button(top, text='清空电脑地图', command=self.clear).pack(side='left', padx=4)
        ttk.Label(top, text='921600 / 8N1').pack(side='left', padx=8)
        self.status = tk.StringVar(value='选择串口并连接；要与板端从头对应，请连接后重启瑞萨板。')
        ttk.Label(self.win, textvariable=self.status).pack(fill='x', padx=8)
        self.fig = Figure(figsize=(7, 7)); self.ax = self.fig.add_subplot(111)
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
        self.grid = Grid(self.lib, defaults()['map']); self.count = 0; self.last = None; self.gaps = 0; self.synced = False
        self.draw(None)

    def connect(self):
        if self.device: return
        try:
            import serial
            self.device = serial.Serial(self.port.get(), 921600, timeout=0.1)
        except Exception as exc:
            messagebox.showerror('串口连接失败', str(exc), parent=self.win); return
        self.stop.clear(); self.parser = StreamParser(); self.queue = queue.Queue(maxsize=2000); self.clear()
        self.worker = threading.Thread(target=self.read_loop, daemon=True); self.worker.start()
        self.status.set('已连接。请重启瑞萨板，以便电脑与板端从空地图同步开始。')

    def read_loop(self):
        try:
            while not self.stop.is_set():
                data = self.device.read(1024)
                for frame in self.parser.feed(data):
                    try: self.queue.put_nowait(frame)
                    except queue.Full:
                        self.queue.get_nowait(); self.queue.put_nowait(('error', '电脑处理队列已满，地图可能不一致')); return
        except Exception as exc:
            if not self.stop.is_set(): self.queue.put(('error', str(exc)))

    def tick(self):
        if not self.win.winfo_exists(): return
        pose = None
        for _ in range(500):
            try: item = self.queue.get_nowait()
            except queue.Empty: break
            if item[0] == 'error': self.status.set('串口异常：' + item[1]); self.disconnect(); break
            seq, pose, rays = item
            if self.last is None: self.synced = (seq == 1)
            if self.last is not None and seq != ((self.last + 1) & 0xffffffff):
                if seq == 1: self.clear(); self.synced = True
                else: self.gaps += 1; self.synced = False
            self.grid.update(pose, rays, seq); self.last = seq; self.count += 1
        if pose is not None:
            self.draw(pose)
            sync = '从首帧开始' if self.synced else '未从首帧开始或有丢帧，地图可能不一致'
            self.status.set(f'{sync}；已处理 {self.count} 包，板端序号 {self.last}，间断 {self.gaps}，CRC/格式错误 {self.parser.bad}')
        self.win.after(100, self.tick)

    def draw(self, pose):
        grid = self.grid.snapshot(); values = np.where(grid['seen'], 1.0 / (1.0 + np.exp(grid['log'])), 0.5)
        self.ax.clear(); self.ax.imshow(values, origin='lower', extent=(-2.5,2.5,-2.5,2.5), cmap='gray', vmin=0, vmax=1)
        if pose: self.ax.plot(pose[0], pose[1], 'ro', markersize=4); self.ax.arrow(pose[0],pose[1],0.2*math.cos(pose[2]),0.2*math.sin(pose[2]),color='red',head_width=.06)
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
