"""有限行程直行实验与日志采集。需要先烧录匹配固件，断开 Ozone 调试连接。

安装依赖：python -m pip install -r tools/imu/requirements.txt
交互输入：python tools/odometry/run_straight.py --resume
参数输入：python tools/odometry/run_straight.py --speed 0.10 --x 1 --y 0 --distance 0.60 --resume
离线演示：python tools/odometry/run_straight.py --simulate --speed 0.10 --x 1 --y 0 --distance 0.10
恢复手柄：python tools/odometry/run_straight.py --release --resume

速度单位 m/s；方向默认使用里程计固定坐标系，--frame start 改为相对于本次开始时车头。
差速底盘先转向，再前进，不能横向平移。
默认按 行程/速度 的时间停车；--stop-mode odometry 改为按编码器距离停车。
完成/中止后保持刹车，--release 显式交还手柄。Ctrl+C 请求中止，断开主机后固件超时制动。
motion.csv 为 100Hz 控制记录，imu_samples.csv 为逐样本原始/滤波 IMU；两者通过 tick_ms 关联。
tick 是发布时刻，不是 IMU 硬件采样时间戳。实测距离应另填，不能把指令距离当成真值。
"""
import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools/imu"))
from capture_imu import (JLinkMemory, BufferReader, SimulatedMemory, Statistics,
                         find_symbol, verify_firmware, read_factory_timing, STATE_NAMES)

HEADER = struct.Struct("<8I")
RECORD = struct.Struct("<IIqqII10fII4fIiII")
COMMAND = struct.Struct("<III3fII")
U32 = struct.Struct("<I")
FIELDS = ("ticket tick_ms left_count right_count state direction dt x y heading linear_speed "
          "angular_speed left_target right_target left_speed right_speed left_pwm right_pwm "
          "distance_delta heading_delta progress_m target_heading imu_stream imu_status "
          "left_state right_state").split()
STATES = ("idle", "turn", "settle", "drive", "stopping", "done", "aborted", "error")
ERRORS = ("none", "bad_command", "heartbeat_lost", "timeout", "L2_estop", "invalid_odometry")
PID_FIELDS = "kp ki kd kf i_max out_max dead_zone angle_max i_variable_min i_variable_max".split()


class HardwareMemory(JLinkMemory):
    def write(self, address, data):
        if address % 4 or len(data) % 4:
            raise ValueError("RAM 命令必须按 32 位对齐")
        words = list(struct.unpack(f"<{len(data)//4}I", data))
        if self.link.memory_write32(address, words) != len(words):
            raise RuntimeError("SWD RAM 命令写入不完整")


class RunReader:
    def __init__(self, memory, address):
        self.memory, self.address = memory, address
        header = self.header()
        if header[:4] != (0x52554E31, 1, 32, RECORD.size):
            raise RuntimeError("实验日志格式不匹配，请烧录本次编译的完整固件。")
        self.cursor = self.last_count = header[4]
        self.dropped = self.retries = 0

    def header(self):
        header = HEADER.unpack(self.memory.read(self.address, HEADER.size))
        if header[6] >= len(STATES) or header[7] >= len(ERRORS):
            raise RuntimeError("实验日志头的状态无效，请核对固件和 ELF")
        return header

    def samples(self):
        count = self.header()[4]
        if count < self.last_count:
            raise RuntimeError("MCU 已复位，停止本次实验。")
        self.last_count = count
        available = count - self.cursor
        if available >= 32:
            lost = available - 31
            self.dropped += lost
            self.cursor += lost
            available -= lost
        if not available:
            return []
        index = self.cursor % 32
        first = min(available, 32 - index)
        raw = self.memory.read(self.address + 32 + index * RECORD.size, first * RECORD.size)
        if first < available:
            raw += self.memory.read(self.address + 32, (available-first) * RECORD.size)
        end = self.header()[4]
        if end < count:
            raise RuntimeError("读取期间 MCU 复位。")
        rows = list(RECORD.iter_unpack(raw))
        if end - self.cursor >= 32 or any(row[0] != self.cursor+i+1 for i, row in enumerate(rows)):
            self.retries += 1
            return []
        for row in rows:
            values = dict(zip(FIELDS, row))
            if values["state"] >= len(STATES) or not all(
                    math.isfinite(values[name]) for name in FIELDS[6:16] + FIELDS[18:22]):
                raise RuntimeError("实验记录中的状态或浮点数据异常。")
        self.cursor = count
        return [dict(zip(FIELDS, row)) for row in rows]


def send_command(memory, address, action, args):
    previous = U32.unpack(memory.read(address, 4))[0]
    guard = ((previous | 1) + 1) & 0xFFFFFFFF
    packet = COMMAND.pack(guard, 0, action, args.speed, math.atan2(args.y, args.x),
                          args.distance, int(args.stop_mode == "odometry"),
                          int(getattr(args, "frame", "odometry") == "odometry"))
    memory.write(address, U32.pack((guard - 1) & 0xFFFFFFFF))
    memory.write(address + 8, packet[8:])
    memory.write(address, U32.pack(guard))
    return guard


def configuration(memory, symbols):
    geometry = struct.unpack("<3f2i", memory.read(symbols["odometry_config"], 20))
    limits = struct.unpack("<4f2I", memory.read(symbols["test_run_config"], 24))
    counts = struct.unpack("<f", memory.read(symbols["odometry_counts_per_rev"], 4))[0]
    if not math.isfinite(counts) or counts <= 0:
        raise RuntimeError("每圈计数系数无效")
    result = {"odometry": dict(zip(("left_radius_m", "right_radius_m", "wheel_track_m",
                                   "left_direction", "right_direction"), geometry)),
              "run_controller": dict(zip(("heading_kp", "max_turn_rate", "heading_tolerance",
                                           "max_speed_mps", "heartbeat_ms", "timeout_ms"), limits)),
              "counts_per_wheel_rev": counts}
    for side in ("left", "right"):
        address = U32.unpack(memory.read(symbols[side + "_motor_pid"], 4))[0]
        if not 0x20000000 <= address <= 0x20005000 - 44:
            raise RuntimeError("电机 PID 尚未初始化，请先复位运行固件。")
        values = struct.unpack("<10fB3x", memory.read(address, 44))
        if not all(math.isfinite(v) for v in values[:10]):
            raise RuntimeError("PID 参数含非有限数值，拒绝启动实验。")
        result[side + "_pid"] = dict(zip(PID_FIELDS + ["d_first"], values))
    for value in (*geometry[:3], *limits[:4]):
        if not math.isfinite(value) or value <= 0:
            raise RuntimeError("轮径、轮距或实验参数无效，请先配置固件中的 RAM 参数。")
    return result


def summarize(records, metadata, reader, imu, stats, error, header):
    drive = next((i for i, r in enumerate(records) if r["state"] == 3), None)
    result = {"mode": metadata["mode"], "state": STATES[header[6]], "firmware_error": ERRORS[header[7]],
              "error": error, "motion_samples": len(records), "motion_host_dropped": reader.dropped,
              "motion_retries": reader.retries, "imu_host_dropped": imu.host_dropped if imu else None,
              "imu_retries": imu.retries if imu else None,
              "imu_accel": stats[0].result(), "imu_gyro": stats[1].result(),
              "requested_distance_m": metadata["distance_m"], "stop_mode": metadata["stop_mode"],
              "actual_distance_m": None,
              "note": "指令值不是实测真值；按里程计距离停车会让内部误差看似很小，仍需尺量。"}
    if drive is not None:
        first, last = records[drive], records[-1]
        dx, dy = last["x"]-first["x"], last["y"]-first["y"]
        heading = last["target_heading"]
        geometry = metadata["configuration"]["odometry"]
        distance = math.pi / metadata["configuration"]["counts_per_wheel_rev"] * (
            (last["left_count"]-first["left_count"]) * geometry["left_direction"] * geometry["left_radius_m"] +
            (last["right_count"]-first["right_count"]) * geometry["right_direction"] * geometry["right_radius_m"])
        result.update({"drive_start": first, "final": last, "encoder_distance_m": distance,
                       "along_direction_m": dx*math.cos(heading)+dy*math.sin(heading),
                       "cross_direction_m": -dx*math.sin(heading)+dy*math.cos(heading),
                       "heading_error_deg": math.degrees(math.atan2(math.sin(last["heading"]-heading),
                                                                       math.cos(last["heading"]-heading)))})
    return result


def capture(memory, symbols, args, output, metadata):
    reader = RunReader(memory, symbols["test_run_log"])
    imu = BufferReader(memory, symbols["imu_log_buffer"]) if symbols.get("imu_log_buffer") is not None else None
    if reader.header()[6] not in (0, 5, 6, 7):
        raise RuntimeError("已有实验正在运行；本脚本不会覆盖其命令。")
    records, stats = [], [Statistics(), Statistics()]
    started, heartbeat, guard, armed = time.monotonic(), 0, None, False
    error, terminal_tick = None, None
    with (output / "motion.csv").open("w", newline="", encoding="utf-8") as mf, \
         (output / "imu_samples.csv").open("w", newline="", encoding="utf-8") as imf, \
         (output / "imu_state.csv").open("w", newline="", encoding="utf-8") as sf:
        motion, samples, states = csv.writer(mf), csv.writer(imf), csv.writer(sf)
        motion.writerow(["host_elapsed_s", *FIELDS])
        samples.writerow(["host_elapsed_s", "ticket", "sensor", "sequence", "stream_id", "read_tick_ms",
                          "sample_dt_s", "x", "y", "z", "unit", "filtered_x", "filtered_y", "filtered_z"])
        states.writerow(["host_elapsed_s", *STATE_NAMES])

        def poll():
            nonlocal heartbeat
            elapsed = time.monotonic()-started
            if isinstance(memory, SimulatedRun):
                memory.advance(elapsed)
            heartbeat = (heartbeat + 1) & 0xFFFFFFFF
            memory.write(symbols["test_run_command"]+4, U32.pack(heartbeat))
            for row in reader.samples():
                records.append(row)
                motion.writerow([f"{elapsed:.6f}", *row.values()])
            if imu:
                for ticket, kind, seq, stream, tick, dt, x, y, z, fx, fy, fz in imu.samples():
                    samples.writerow([f"{elapsed:.6f}", ticket, "accel" if kind == 0 else "gyro",
                                      seq, stream, tick, dt, x, y, z, "m/s^2" if kind == 0 else "rad/s", fx, fy, fz])
                    stats[kind].add((x, y, z))
                state = imu.state()
                if state:
                    states.writerow([f"{elapsed:.6f}", *state.values()])
            return reader.header()

        try:
            # 先建立日志，再提交运动命令；通信故障由 finally 和固件心跳共同处理。
            poll()
            armed = True
            guard = send_command(memory, symbols["test_run_command"], 1, args)
            last_progress = last_flush = 0.0
            while True:
                header = poll()
                elapsed = time.monotonic()-started
                if header[5] != guard:
                    if elapsed > 2:
                        raise RuntimeError("固件未确认命令，已请求停止。")
                elif header[6] >= 5:
                    if header[6] != 5:
                        raise RuntimeError(f"实验中止：{STATES[header[6]]} / {ERRORS[header[7]]}")
                    terminal_tick = terminal_tick or time.monotonic()
                    if time.monotonic()-terminal_tick >= 0.3:
                        break
                if elapsed > 125:
                    raise RuntimeError("主机实验超时。")
                if elapsed-last_progress >= 1:
                    latest = records[-1] if records else {}
                    print(f"{elapsed:.1f}s {STATES[header[6]]}: 距离={latest.get('progress_m', 0):.3f}m, "
                          f"x={latest.get('x', 0):.3f}, y={latest.get('y', 0):.3f}, 丢记录={reader.dropped}", flush=True)
                    last_progress = elapsed
                if elapsed-last_flush >= 1:
                    mf.flush(); imf.flush(); sf.flush()
                    last_flush = elapsed
                time.sleep(0.02)
        except KeyboardInterrupt:
            error = "用户 Ctrl+C 中止"
        except Exception as exc:
            error = str(exc)
        finally:
            if armed and terminal_tick is None:
                try:
                    try:
                        stopped = reader.header()[6] >= 5
                    except Exception:
                        stopped = False
                    if not stopped:
                        send_command(memory, symbols["test_run_command"], 2, args)
                    stop_until = time.monotonic()+0.3
                    while time.monotonic() < stop_until:
                        poll()
                        time.sleep(0.02)
                except Exception as exc:
                    error = (error or "") + f"；停止命令发送/确认失败：{exc}，固件依靠心跳超时制动"
            try:
                header = reader.header()
            except Exception:
                header = (0, 0, 0, 0, 0, 0, 7, 2)
            result = summarize(records, metadata, reader, imu, stats, error, header)
            (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
    return result


class SimulatedRun:
    """仅验证命令和日志管线的理想模型，不代表真实控制或 IMU 精度。"""
    symbols = {"test_run_log": 0x10000, "test_run_command": 0x20000, "imu_log_buffer": 0}

    def __init__(self):
        self.imu = SimulatedMemory()
        self.command = bytearray(COMMAND.size)
        self.header = [0x52554E31, 1, 32, RECORD.size, 0, 0, 0, 0]
        self.ring = bytearray(32*RECORD.size)
        self.t = self.phase_tick = self.drive_tick = 0.0
        self.x = self.y = self.h = self.progress = self.target = 0.0
        self.left = self.right = 0
        self.speed, self.distance, self.stop_mode = 0.1, 0.1, 0
        self.last_hb, self.hb_tick = 0, 0.0
        self.v = self.w = 0.0

    def write(self, address, data):
        offset = address-self.symbols["test_run_command"]
        if not 0 <= offset <= COMMAND.size-len(data):
            raise ValueError("模拟写地址越界")
        self.command[offset:offset+len(data)] = data

    def read(self, address, size):
        if address < self.symbols["test_run_log"]:
            return self.imu.read(address, size)
        if address >= self.symbols["test_run_command"]:
            offset = address-self.symbols["test_run_command"]
            return self.command[offset:offset+size]
        offset = address-self.symbols["test_run_log"]
        return (HEADER.pack(*self.header)+self.ring)[offset:offset+size]

    def advance(self, elapsed):
        self.imu.advance(elapsed)
        while self.t+0.01 <= elapsed:
            self.t += 0.01
            actual_v, actual_w = self.v, self.w
            self.x += actual_v*math.cos(self.h+actual_w*0.005)*0.01
            self.y += actual_v*math.sin(self.h+actual_w*0.005)*0.01
            self.h = math.atan2(math.sin(self.h+actual_w*0.01), math.cos(self.h+actual_w*0.01))
            actual_left, actual_right = (actual_v-actual_w*0.075)/0.03, (actual_v+actual_w*0.075)/0.03
            self.left += round(actual_left*0.01*60000/(2*math.pi))
            self.right -= round(actual_right*0.01*60000/(2*math.pi))
            guard, hb, action, speed, angle, distance, mode, frame = COMMAND.unpack(self.command)
            if hb != self.last_hb:
                self.last_hb, self.hb_tick = hb, self.t
            if guard != self.header[5] and guard % 2 == 0:
                self.header[5] = guard
                if action == 1:
                    self.speed, self.distance, self.stop_mode = speed, distance, mode
                    self.target = angle+(self.h if frame == 0 else 0)
                    self.progress = 0
                    self.header[6:8] = [1, 0]
                    self.hb_tick = self.t
                elif action == 2:
                    self.header[6:8] = [6, 0]
                elif action == 3:
                    self.header[6:8] = [0, 0]
            state = self.header[6]
            v = w = 0.0
            error = math.atan2(math.sin(self.target-self.h), math.cos(self.target-self.h))
            if 0 < state < 5 and self.t-self.hb_tick > 0.5:
                self.header[6:8] = [7, 2]
            elif state == 1:
                if abs(error) < 0.03:
                    self.header[6], self.phase_tick = 2, self.t
                else:
                    w = max(-0.6, min(0.6, 2*error))
            elif state == 2 and self.t-self.phase_tick >= 0.3:
                self.header[6], self.drive_tick = 3, self.t
                v, w = self.speed, max(-0.6, min(0.6, 2*error))
            elif state == 3:
                self.progress += actual_v*0.01
                finished = self.t-self.drive_tick >= self.distance/self.speed if mode == 0 else self.progress >= self.distance
                if finished:
                    self.header[6], self.phase_tick = 4, self.t
                else:
                    v, w = self.speed, max(-0.6, min(0.6, 2*error))
            elif state == 4 and self.t-self.phase_tick >= 1:
                self.header[6] = 5
            self.v, self.w = v, w
            left, right = (v-w*0.075)/0.03, (v+w*0.075)/0.03
            count = self.header[4]
            row = [count+1, round(self.t*1000), self.left, self.right, self.header[6], 0,
                   0.01, self.x, self.y, self.h, actual_v, actual_w, abs(left), abs(right), abs(actual_left), abs(actual_right),
                   500 if left else 0, 500 if right else 0, actual_v*0.01, actual_w*0.01, self.progress, self.target,
                   0, 1, 0 if left > 0 else 1 if left < 0 else 3, 0 if right > 0 else 1 if right < 0 else 3]
            start = count % 32 * RECORD.size
            self.ring[start:start+RECORD.size] = RECORD.pack(*row)
            self.header[4] += 1

    def close(self):
        pass


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--speed", type=float, help="直行速度，m/s，固件默认上限 0.3")
    parser.add_argument("--x", type=float, help="方向 X 分量，脚本会归一化")
    parser.add_argument("--y", type=float, help="方向 Y 分量")
    parser.add_argument("--distance", type=float, help="实验行程，m")
    parser.add_argument("--stop-mode", choices=["time", "odometry"], default="time")
    parser.add_argument("--frame", choices=["odometry", "start"], default="odometry",
                        help="方向坐标系：默认 odometry 固定坐标系；start 为本次开始时车头坐标系")
    parser.add_argument("--actual-distance", type=float, help="独立尺量的实际直行距离，m；可在采集后另行记录")
    parser.add_argument("--elf", type=Path, default=ROOT / "STM32_F103/build/Debug/STM32_F103.elf")
    parser.add_argument("--nm"); parser.add_argument("--dll"); parser.add_argument("--serial", type=int)
    parser.add_argument("--device", default="STM32F103C8")
    parser.add_argument("--swd-speed", dest="swd_speed", type=int, default=1000)
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--release", action="store_true", help="停车后显式交还手柄控制")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--label", default="")
    args = parser.parse_args(argv)
    interactive = all(getattr(args, key) is None for key in ("speed", "x", "y", "distance")) and not args.simulate and not args.release
    defaults = {"speed": 0.10, "x": 1.0, "y": 0.0, "distance": 0.60}
    labels = {"speed": "速度 m/s", "x": "方向 X 分量", "y": "方向 Y 分量", "distance": "行程 m"}
    memory = None
    try:
        if interactive:
            print(f"方向坐标系：{args.frame}；先转向再直行；停车依据：{args.stop_mode}。")
        for key, default in defaults.items():
            if getattr(args, key) is None:
                text = input(f"{labels[key]} [{default}]：").strip() if interactive else ""
                setattr(args, key, float(text) if text else default)
        if not all(math.isfinite(getattr(args, k)) for k in defaults):
            raise ValueError("速度、方向和行程必须是有限数值")
        norm = math.hypot(args.x, args.y)
        if not math.isfinite(norm) or norm <= 0 or args.speed <= 0 or args.distance <= 0 or args.swd_speed <= 0:
            raise ValueError("方向不能为零，速度、行程和 SWD 速率必须为正")
        if args.actual_distance is not None and (not math.isfinite(args.actual_distance) or args.actual_distance <= 0):
            raise ValueError("实测距离必须为正数")
        args.x, args.y = args.x/norm, args.y/norm
        if args.simulate:
            memory, symbols = SimulatedRun(), SimulatedRun.symbols
            config = {"odometry": {"left_radius_m": 0.03, "right_radius_m": 0.03, "wheel_track_m": 0.15,
                                    "left_direction": 1, "right_direction": -1}, "counts_per_wheel_rev": 60000,
                      "run_controller": {"max_speed_mps": 0.3, "timeout_ms": 120000}}
            timing = None
        else:
            names = ("test_run_log", "test_run_command", "test_run_config", "odometry_config",
                     "odometry_counts_per_rev", "left_motor_pid", "right_motor_pid")
            symbols = {name: find_symbol(args.elf, args.nm, name) for name in names}
            symbols["imu_log_buffer"] = find_symbol(args.elf, args.nm, "imu_log_buffer", required=False)
            link_args = argparse.Namespace(**vars(args))
            link_args.speed = args.swd_speed
            memory = HardwareMemory(link_args)
            print("核对目标 Flash 与 ELF……", flush=True)
            verify_firmware(memory, args.elf)
            ready_until = time.monotonic()+2
            while HEADER.unpack(memory.read(symbols["test_run_log"], 32))[0] != 0x52554E31:
                if time.monotonic() >= ready_until:
                    raise RuntimeError("实验模块未初始化，请先复位运行固件。")
                time.sleep(0.02)
            config = {} if args.release else configuration(memory, symbols)
            timing = None if args.release else read_factory_timing(
                memory, find_symbol(args.elf, args.nm, "lsm6dsr_timing", False),
                find_symbol(args.elf, args.nm, "lsm6dsr_factory_dt_enabled", False))
        reader = RunReader(memory, symbols["test_run_log"])
        if args.release:
            if reader.header()[6] not in (0, 5, 6, 7):
                raise RuntimeError("实验尚未结束，请先 L2 或 Ctrl+C 中止。")
            guard = send_command(memory, symbols["test_run_command"], 3, args)
            until = time.monotonic()+1
            while time.monotonic() < until:
                if args.simulate:
                    memory.advance(0.1)
                header = reader.header()
                if header[5] == guard and header[6] == 0:
                    print("已释放，恢复原有手柄控制。")
                    return 0
                time.sleep(0.02)
            raise RuntimeError("固件没有确认释放控制权")
        limits = config["run_controller"]
        if args.speed > limits["max_speed_mps"] or args.distance/args.speed*1000 >= limits["timeout_ms"]:
            raise ValueError("速度超过固件上限，或直行时间超过实验总超时")
        output = args.output or ROOT / "logs/odometry" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output.mkdir(parents=True, exist_ok=False)
        metadata = {"mode": "simulation" if args.simulate else "hardware", "label": args.label,
                    "started_at": datetime.now().astimezone().isoformat(), "speed_mps": args.speed,
                    "direction_unit": [args.x, args.y], "velocity_xy_mps": [args.speed*args.x, args.speed*args.y],
                    "direction_frame": args.frame, "pose_frame": "odometry",
                    "distance_m": args.distance, "stop_mode": args.stop_mode, "configuration": config,
                    "factory_timing": timing, "elf": str(args.elf.resolve()),
                    "motion_state_names": dict(enumerate(STATES)), "error_names": dict(enumerate(ERRORS)),
                    "motor_state_names": {0: "forward", 1: "backward", 2: "coast", 3: "brake"},
                    "motion_direction_names": {0: "forward", 1: "backward", 2: "left", 3: "right", 4: "stop", 5: "brake"},
                    "elf_sha256": None if args.simulate else hashlib.sha256(args.elf.read_bytes()).hexdigest(),
                    "transport": "simulation" if args.simulate else "J-Link SWD",
                    "imu_used_for_control": False if args.simulate else None,
                    "imu_control_note": "hardware: IMU may contribute to odometry heading after calibration; see firmware odometry_data.imu_used",
                    "imu_frame": "samples: sensor XYZ; body mapping: see firmware app_attitude.c",
                    "imu_units": {"accel": "m/s^2, includes gravity", "gyro": "rad/s"},
                    "timing": "motion.tick_ms 与 imu.read_tick_ms 均为发布时刻；不代表 IMU 硬件采样时间戳"}
        (output / "metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"模式：{metadata['mode']}，方向=({args.x:.4f},{args.y:.4f})，速度={args.speed}m/s，"
              f"方向坐标系={args.frame}，停车依据={args.stop_mode}，日志：{output}", flush=True)
        result = capture(memory, symbols, args, output, metadata)
        actual = args.actual_distance
        if interactive and result["state"] == "done" and actual is None:
            text = input("独立测量的实际直行距离 m（留空跳过）：").strip()
            if text:
                actual = float(text)
                if not math.isfinite(actual) or actual <= 0:
                    raise ValueError("实测距离必须为正数；原始日志已保存")
        if actual is not None:
            result["actual_distance_m"] = actual
            measured = result.get("encoder_distance_m", 0)
            if result["state"] == "done" and not result["error"]:
                result["radius_scale_suggestion"] = actual/measured if measured > 0 else None
                result["distance_error_percent"] = 100*(measured/actual-1)
            (output / "summary.json").write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")
        print(f"保存完成：{output}；状态={result['state']}，误差原因={result['firmware_error']}")
        print("结束后仍保持刹车；恢复手柄请运行 --release。")
        if result.get("encoder_distance_m") is not None:
            print(f"编码器估计距离={result['encoder_distance_m']:.4f}m，横向偏移={result['cross_direction_m']:.4f}m")
        return 1 if result["error"] else 0
    except (Exception, KeyboardInterrupt) as exc:
        print(f"实验失败/中止：{exc}", file=sys.stderr)
        return 1
    finally:
        if memory is not None:
            memory.close()


if __name__ == "__main__":
    sys.exit(main())
