"""通过 J-Link/SWD 保存逐样本 IMU 日志，不需要串口。

首次安装：python -m pip install -r tools/imu/requirements.txt
先烧录最新 Debug 固件，关闭 Ozone 的调试连接，再执行：
    python tools/imu/capture_imu.py --duration 30 --label static
离线验证：python tools/imu/capture_imu.py --simulate --duration 2

默认日志位置为仓库 logs/imu/时间戳：samples.csv、state.csv、metadata.json、summary.json。
每条样本是加速度或陀螺仪三轴；单位分别为 m/s²、rad/s。dt 来自标称 ODR。
read_tick_ms 是固件发布时刻，host_elapsed_s 是主机读取时刻，都不是硬件采样时间戳。
state.csv 是主机轮询时的状态快照，不保证记录每一次任务状态变化。
采集时目标必须运行；不主动复位或烧录。--resume 可继续已暂停的 CPU。
"""

import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
HEADER = struct.Struct("<7IiIi11If")
RECORD = struct.Struct("<5I4f")
U32 = struct.Struct("<I")
MAGIC = 0x494D554C
STATE_NAMES = (
    "magic version capacity record_size write_count status_guard tick_ms init_result "
    "who_am_i status level_before level_after accel_count gyro_count accel_total "
    "gyro_total overrun_count io_error_count tag_error_count stream_id update_time_ms task_dt_s"
).split()


def find_symbol(elf, nm=None, symbol="imu_log_buffer", required=True):
    """从匹配固件的 ELF 找符号，避免硬编码会随链接变化的 RAM 地址。"""
    nm = nm or shutil.which("arm-none-eabi-nm")
    if not nm:
        raise RuntimeError("找不到 arm-none-eabi-nm，请加入 PATH 或通过 --nm 指定。")
    result = subprocess.run([str(nm), "--defined-only", str(elf)],
                            check=True, capture_output=True, text=True)
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2] == symbol:
            return int(parts[0], 16)
    if required:
        raise RuntimeError(f"ELF 中没有 {symbol}，请编译并烧录包含日志模块的固件。")
    return None


def read_factory_timing(memory, address):
    """读取初始化后固定的诊断数据，旧固件没有此符号时仍可采集。"""
    if address is None:
        return None
    fine, accel_hz, gyro_hz, accel_dt, gyro_dt, valid = struct.unpack(
        "<i4fI", memory.read(address, 24))
    if valid != 1:
        return None
    if not -128 <= fine <= 127 or not all(math.isfinite(v) and v > 0
                                         for v in (accel_hz, gyro_hz, accel_dt, gyro_dt)):
        raise RuntimeError("工厂频率诊断数据异常，请确认初始化和固件版本。")
    return {"freq_fine": fine, "accel_odr_hz_est": accel_hz, "gyro_odr_hz_est": gyro_hz,
            "accel_dt_s_est": accel_dt, "gyro_dt_s_est": gyro_dt,
            "applied_to_sample_dt": False}


def flash_segments(elf):
    """取得有文件内容的分配节及其 LMA，不校验加载段之间的对齐填充。"""
    data = Path(elf).read_bytes()
    if len(data) < 52 or data[:6] != b"\x7fELF\x01\x01":
        raise RuntimeError("需要 STM32 的小端 ELF32 固件文件。")
    fields = struct.unpack_from("<16sHHIIIIIHHHHHH", data)
    if fields[2] != 40 or fields[9] != 32 or fields[11] != 40:
        raise RuntimeError("ELF 架构或加载段格式不符合 ARM ELF32。")
    loads = []
    for i in range(fields[10]):
        kind, offset, virtual, physical, size, _, _, _ = struct.unpack_from(
            "<8I", data, fields[5] + i * fields[9])
        if kind == 1 and size:
            loads.append((offset, virtual, physical, size))
    result = []
    for i in range(fields[12]):
        _, kind, flags, virtual, offset, size, _, _, _, _ = struct.unpack_from(
            "<10I", data, fields[6] + i * fields[11])
        # SHF_ALLOC=2；SHT_NOBITS=8（.bss）没有待烧录的文件内容。
        if not flags & 2 or kind == 8 or not size:
            continue
        load = next((item for item in loads
                     if item[0] <= offset and offset + size <= item[0] + item[3]
                     and virtual - item[1] == offset - item[0]), None)
        if load is None:
            continue
        address = load[2] + virtual - load[1]
        if 0x08000000 <= address and address + size <= 0x08010000:
            if offset + size > len(data):
                raise RuntimeError("ELF 的 Flash 加载段超出文件范围。")
            result.append((address, data[offset:offset + size]))
    if not result:
        raise RuntimeError("ELF 中没有 STM32F103C8 的 Flash 加载段。")
    return sorted(result)


def verify_firmware(memory, elf):
    """不暂停、不写入目标，对比全部 Flash 加载内容，避免用新 ELF 解析旧 RAM。"""
    checked = 0
    for address, expected in flash_segments(elf):
        for offset in range(0, len(expected), 1024):
            chunk = expected[offset:offset + 1024]
            location = address + offset
            aligned = location & ~3
            prefix = location - aligned
            size = (prefix + len(chunk) + 3) & ~3
            actual = memory.read(aligned, size)[prefix:prefix + len(chunk)]
            if actual != chunk:
                index = next((i for i, pair in enumerate(zip(actual, chunk))
                              if pair[0] != pair[1]), len(actual))
                location = address + offset + index
                raise RuntimeError(
                    f"目标 Flash 与 ELF 不一致，首个差异位于 0x{location:08X}。\n"
                    f"ELF 期望：{chunk[index:index+16].hex(' ')}\n"
                    f"芯片读回：{actual[index:index+16].hex(' ')}\n"
                    f"请重新下载完整固件：{Path(elf).resolve()}\n"
                    "仅关闭 Ozone 或重新加载符号不会更新 Flash。下载完成后复位运行，再断开调试连接。")
            checked += len(chunk)
    return checked


class JLinkMemory:
    def __init__(self, args):
        try:
            import pylink
        except ImportError as exc:
            raise RuntimeError("请先执行 python -m pip install -r tools/imu/requirements.txt") from exc
        library = pylink.library.Library(dllpath=args.dll) if args.dll else None
        self.link = pylink.JLink(lib=library) if library else pylink.JLink()
        try:
            self.link.open(serial_no=args.serial)
            self.link.set_tif(pylink.enums.JLinkInterfaces.SWD)
            self.link.connect(args.device, speed=args.speed)
            if self.link.halted():
                if not args.resume:
                    raise RuntimeError("CPU 已暂停。先继续运行，或使用 --resume；本脚本不会复位目标。")
                self.link.restart()
        except Exception:
            self.link.close()
            raise

    def read(self, address, size):
        if address % 4 or size % 4:
            raise ValueError("日志读取必须按 32 位对齐")
        words = self.link.memory_read32(address, size // 4)
        if len(words) * 4 != size:
            raise RuntimeError("SWD 内存读取不完整")
        return struct.pack(f"<{len(words)}I", *words)

    def close(self):
        self.link.close()


class SimulatedMemory:
    """确定性离线数据源，仅用于验证脚本，绝不标为实板数据。"""
    def __init__(self):
        self.header = [MAGIC, 1, 64, RECORD.size, 0, 0, 0, 0, 0x6B, 1,
                       2, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 0.010]
        self.records = bytearray(64 * RECORD.size)
        self.generated = 0

    def advance(self, elapsed):
        while self.generated < int(elapsed * 104):
            self.generated += 1
            for kind in (0, 1):
                count = self.header[4]
                noise = 0.002 * math.sin(self.generated * 0.7)
                xyz = (noise, -noise, 9.80665 + noise) if kind == 0 else (noise, noise, -noise)
                raw = RECORD.pack(count + 1, kind, self.generated, 0,
                                  round(self.generated * 1000 / 104),
                                  0.0 if self.generated == 1 else 1 / 104, *xyz)
                start = count % 64 * RECORD.size
                self.records[start:start + RECORD.size] = raw
                self.header[4] += 1
            self.header[5] += 2
            self.header[6] = round(self.generated * 1000 / 104)
            self.header[14] = self.header[15] = self.generated

    def read(self, address, size):
        return (HEADER.pack(*self.header) + self.records)[address:address + size]

    def close(self):
        pass


class BufferReader:
    def __init__(self, memory, address):
        self.memory, self.address = memory, address
        fields = HEADER.unpack(self.read(0, HEADER.size))
        if fields[:4] != (MAGIC, 1, 64, RECORD.size):
            raise RuntimeError(
                f"RAM 日志头不匹配，地址 0x{address:08X}。\n"
                f"期望：magic=0x{MAGIC:08X}, version=1, capacity=64, record_size={RECORD.size}\n"
                f"读回：magic=0x{fields[0]:08X}, version={fields[1]}, "
                f"capacity={fields[2]}, record_size={fields[3]}\n"
                "请确认固件已复位运行并完成 C 启动初始化；若 Flash 校验通过，继续排查 RAM 被覆盖。")
        self.capacity = fields[2]
        self.cursor = self.last_write = fields[4]  # 只记录本次启动脚本后的新样本。
        self.host_dropped = 0
        self.retries = 0

    def read(self, offset, size):
        raw = self.memory.read(self.address + offset, size)
        if len(raw) != size:
            raise RuntimeError("日志内存读取不完整")
        return raw

    def count(self):
        return U32.unpack(self.read(16, 4))[0]

    def samples(self):
        """读取已提交记录；前后计数及 ticket 防止返回被覆盖或撕裂的数据。"""
        start = self.count()
        if start < self.last_write:
            raise RuntimeError("MCU 复位或日志计数回绕；停止采集，请开始新日志。")
        self.last_write = start
        available = start - self.cursor
        # 留出一个正在写入的槽位，读得过慢时明确记录丢失数。
        if available >= self.capacity:
            dropped = available - (self.capacity - 1)
            self.host_dropped += dropped
            self.cursor += dropped
            available -= dropped
        if not available:
            return []
        index = self.cursor % self.capacity
        first = min(available, self.capacity - index)
        raw = self.read(HEADER.size + index * RECORD.size, first * RECORD.size)
        if first < available:
            raw += self.read(HEADER.size, (available - first) * RECORD.size)
        end = self.count()
        if end < start:
            raise RuntimeError("读取期间 MCU 复位，停止采集。")
        records = list(RECORD.iter_unpack(raw))
        if end - self.cursor >= self.capacity or any(
                row[0] != self.cursor + i + 1 for i, row in enumerate(records)):
            self.retries += 1
            return []  # 下一轮重新取，绝不把可能被覆盖的记录写入 CSV。
        for row in records:
            if row[1] not in (0, 1) or not all(math.isfinite(v) for v in row[5:]):
                raise RuntimeError("日志样本类型或浮点值异常，请检查 ELF、固件和通信。")
        self.cursor = start
        return records

    def state(self):
        for _ in range(3):
            values = HEADER.unpack(self.read(0, HEADER.size))
            guard = U32.unpack(self.read(20, 4))[0]
            if values[5] == guard and guard % 2 == 0:
                return dict(zip(STATE_NAMES, values))
        return None


class Statistics:
    def __init__(self):
        self.n = 0
        self.mean = [0.0] * 4
        self.m2 = [0.0] * 4

    def add(self, xyz):
        self.n += 1
        for i, value in enumerate((*xyz, math.sqrt(sum(v * v for v in xyz)))):
            delta = value - self.mean[i]
            self.mean[i] += delta / self.n
            self.m2[i] += delta * (value - self.mean[i])

    def result(self):
        return {"samples": self.n, "axes_and_norm_mean": self.mean,
                "axes_and_norm_std": [math.sqrt(max(0, x) / (self.n - 1))
                                      if self.n > 1 else 0 for x in self.m2]}


class TimingStatistics:
    """按数据流和传感器分别估算发布速率，避免把恢复前后数据混在一起。"""
    def __init__(self):
        self.spans = {}

    def add(self, kind, stream, sequence, tick, host):
        point = (sequence, tick, host)
        key = (stream, kind)
        if key not in self.spans:
            self.spans[key] = {"first": point, "last": point, "count": 1, "gaps": 0}
        else:
            span = self.spans[key]
            span["gaps"] += max(0, sequence - span["last"][0] - 1)
            span["last"] = point
            span["count"] += 1

    def result(self, factory):
        result = []
        for (stream, kind), span in sorted(self.spans.items()):
            first, last = span["first"], span["last"]
            delta = last[0] - first[0]
            host_span = last[2] - first[2]
            mcu_span = ((last[1] - first[1]) & 0xFFFFFFFF) / 1000
            host_rate = delta / host_span if delta > 0 and host_span > 0 else None
            mcu_rate = delta / mcu_span if delta > 0 and mcu_span > 0 else None
            sensor = "accel" if kind == 0 else "gyro"
            estimated = factory.get(sensor + "_odr_hz_est") if factory else None
            result.append({"stream_id": stream, "sensor": sensor, "samples": span["count"],
                           "sequence_gaps": span["gaps"], "host_rate_hz": host_rate,
                           "mcu_rate_hz": mcu_rate, "factory_estimated_hz": estimated,
                           "host_interval_s_est": 1 / host_rate if host_rate else None,
                           "host_vs_factory_percent":
                               100 * (host_rate / estimated - 1) if host_rate and estimated else None})
        return result


def capture(memory, address, args, output, metadata):
    reader = BufferReader(memory, address)
    stats = [Statistics(), Statistics()]
    timing_stats = TimingStatistics()
    start = last_progress = last_flush = last_data = time.monotonic()
    last_state = None
    error = None
    with (output / "samples.csv").open("w", newline="", encoding="utf-8") as sample_file, \
            (output / "state.csv").open("w", newline="", encoding="utf-8") as state_file:
        samples, states = csv.writer(sample_file), csv.writer(state_file)
        samples.writerow(["host_elapsed_s", "ticket", "sensor", "sequence", "stream_id",
                          "read_tick_ms", "nominal_dt_s", "x", "y", "z", "unit"])
        states.writerow(["host_elapsed_s", *STATE_NAMES, "host_dropped", "snapshot_retries"])
        try:
            while time.monotonic() - start < args.duration:
                now = time.monotonic()
                if isinstance(memory, SimulatedMemory):
                    memory.advance(now - start)
                rows = reader.samples()
                if rows:
                    last_data = now
                for ticket, kind, sequence, stream, tick, dt, x, y, z in rows:
                    samples.writerow([f"{now - start:.6f}", ticket,
                                      "accel" if kind == 0 else "gyro", sequence, stream,
                                      tick, dt, x, y, z, "m/s^2" if kind == 0 else "rad/s"])
                    stats[kind].add((x, y, z))
                    timing_stats.add(kind, stream, sequence, tick, now - start)
                state = reader.state()
                if state:
                    last_state = state
                    states.writerow([f"{now - start:.6f}", *state.values(),
                                     reader.host_dropped, reader.retries])
                    if state["init_result"] < -1:
                        raise RuntimeError(f"IMU 初始化失败：{state['init_result']}，请检查硬件。")
                if now - last_data > 3:
                    raise RuntimeError("连续 3 秒没有新样本，请检查目标是否运行及初始化状态。")
                if now - last_flush >= 1:
                    sample_file.flush()
                    state_file.flush()
                    last_flush = now
                if now - last_progress >= 1:
                    print(f"{now-start:.1f}s: accel={stats[0].n}, gyro={stats[1].n}, "
                          f"host_dropped={reader.host_dropped}", flush=True)
                    last_progress = now
                time.sleep(args.poll_ms / 1000)
        except KeyboardInterrupt:
            print("\n已停止，正在保存日志。")
        except Exception as exc:
            error = str(exc)
        finally:
            summary = {"mode": metadata["mode"], "elapsed_s": time.monotonic() - start,
                       "accel": stats[0].result(), "gyro": stats[1].result(),
                       "host_dropped": reader.host_dropped, "snapshot_retries": reader.retries,
                       "factory_timing": metadata.get("factory_timing"),
                       "timing_by_stream": timing_stats.result(metadata.get("factory_timing")),
                       "last_state": last_state, "error": error,
                       "note": "统计覆盖整个采集区间；仅静止区间适合分析零偏和静态噪声。"}
            (output / "summary.json").write_text(json.dumps(summary, indent=2, ensure_ascii=False),
                                                 encoding="utf-8")
            for row in summary["timing_by_stream"]:
                if row["host_rate_hz"] is not None:
                    text = f"{row['sensor']} 流 {row['stream_id']}：主机估算 {row['host_rate_hz']:.3f} Hz"
                    if row["factory_estimated_hz"] is not None:
                        text += (f"，工厂值估算 {row['factory_estimated_hz']:.3f} Hz"
                                 f"，偏差 {row['host_vs_factory_percent']:+.2f}%")
                    print(text)
    if error:
        raise RuntimeError(error)
    if not sum(s.n for s in stats):
        raise RuntimeError("未收到样本，本次日志不包含有效采集数据。")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--elf", type=Path, default=ROOT / "STM32_F103/build/Debug/STM32_F103.elf")
    parser.add_argument("--nm", help="arm-none-eabi-nm 可执行文件路径")
    parser.add_argument("--dll", help="JLink_x64.dll 路径，自动发现失败时指定")
    parser.add_argument("--serial", type=int, help="J-Link 序列号，有多个探针时指定")
    parser.add_argument("--device", default="STM32F103C8")
    parser.add_argument("--speed", type=int, default=1000, help="SWD 速率，单位 kHz")
    parser.add_argument("--duration", type=float, default=30, help="采集秒数；Ctrl+C 可提前停止")
    parser.add_argument("--poll-ms", type=float, default=20, help="主机轮询间隔，默认 20ms")
    parser.add_argument("--label", default="", help="实验备注，如 static、rotate_x")
    parser.add_argument("--output", type=Path, help="新建日志目录；已有目录不覆盖")
    parser.add_argument("--resume", action="store_true", help="连接后若 CPU 暂停则继续执行")
    parser.add_argument("--simulate", action="store_true", help="使用离线模拟数据，不连接硬件")
    args = parser.parse_args(argv)
    if (not math.isfinite(args.duration) or args.duration <= 0 or
            not math.isfinite(args.poll_ms) or args.poll_ms <= 0 or args.speed <= 0):
        parser.error("duration、poll-ms、speed 必须为正数")
    memory = None
    try:
        address = 0 if args.simulate else find_symbol(args.elf, args.nm)
        memory = SimulatedMemory() if args.simulate else JLinkMemory(args)
        if not args.simulate:
            print("正在核对目标 Flash 与 ELF（包含 RAM 初值镜像）……", flush=True)
            checked = verify_firmware(memory, args.elf)
            print(f"固件校验通过：{checked} 字节。", flush=True)
        timing_address = None if args.simulate else find_symbol(
            args.elf, args.nm, symbol="lsm6dsr_timing", required=False)
        factory_timing = None if args.simulate else read_factory_timing(memory, timing_address)
        if factory_timing:
            print(f"FREQ_FINE={factory_timing['freq_fine']}，"
                  f"加速度/陀螺仪 ODR 估算为 {factory_timing['accel_odr_hz_est']:.3f}/"
                  f"{factory_timing['gyro_odr_hz_est']:.3f} Hz（尚未应用到样本 dt）。", flush=True)
        elif not args.simulate:
            print("本次没有可用的工厂频率诊断；仍按现有样本 dt 记录。", flush=True)
        output = args.output or ROOT / "logs/imu" / datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        output.mkdir(parents=True, exist_ok=False)
        metadata = {"mode": "simulation" if args.simulate else "hardware",
                    "started_at": datetime.now().astimezone().isoformat(), "label": args.label,
                    "elf": str(args.elf.resolve()), "buffer_address": hex(address),
                    "elf_sha256": None if args.simulate else hashlib.sha256(args.elf.read_bytes()).hexdigest(),
                    "duration_s": args.duration, "poll_ms": args.poll_ms,
                    "factory_timing": factory_timing,
                    "device": args.device, "serial": args.serial, "swd_speed_khz": args.speed,
                    "timing": "nominal_dt_s 是标称间隔；read_tick_ms/host_elapsed_s 不是采样时间戳。"}
        (output / "metadata.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False),
                                              encoding="utf-8")
        print(f"模式：{metadata['mode']}，日志目录：{output}", flush=True)
        capture(memory, address, args, output, metadata)
        print(f"保存完成：{output}")
        return 0
    except Exception as exc:
        print(f"采集失败：{exc}", file=sys.stderr)
        return 1
    finally:
        if memory is not None:
            memory.close()


if __name__ == "__main__":
    sys.exit(main())
