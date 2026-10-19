"""运行：python -m unittest discover -s tools/imu -p test_*.py -v。"""
import argparse
import csv
import json
import tempfile
from pathlib import Path
import unittest

from capture_imu import (BufferReader, HEADER, RECORD, SimulatedMemory, Statistics,
                         capture, find_symbol, flash_segments, verify_firmware,
                         read_factory_timing, TimingStatistics)


class CaptureTests(unittest.TestCase):
    def test_factory_timing_decode_and_rate_comparison(self):
        import struct

        expected = 6667 * (1 + 0.0015 * 36) / 64

        class TimingMemory:
            def read(self, address, size):
                return struct.pack("<i4fI", 36, expected, expected, 1/expected, 1/expected, 1)

        timing = read_factory_timing(TimingMemory(), 0)
        self.assertEqual(timing["freq_fine"], 36)
        self.assertAlmostEqual(timing["accel_odr_hz_est"], expected, places=4)
        self.assertFalse(timing["applied_to_sample_dt"])
        rates = TimingStatistics()
        rates.add(0, 0, 100, 1000, 1.0)
        rates.add(0, 0, 101, 1010, 1.01)
        rates.add(0, 0, 102, 1020, 1.02)
        rates.add(0, 1, 103, 2000, 2.0)  # 新流不能跨中断计算速率。
        result = rates.result(timing)
        self.assertAlmostEqual(result[0]["host_rate_hz"], 100)
        self.assertAlmostEqual(result[0]["mcu_rate_hz"], 100)
        self.assertEqual(result[0]["sequence_gaps"], 0)
        self.assertIsNone(result[1]["host_rate_hz"])

    def test_layout_and_sample_order(self):
        self.assertEqual(HEADER.size, 88)
        self.assertEqual(RECORD.size, 36)
        memory = SimulatedMemory()
        reader = BufferReader(memory, 0)
        memory.advance(0.02)
        rows = reader.samples()
        self.assertEqual([row[1] for row in rows], [0, 1, 0, 1])
        self.assertEqual([row[2] for row in rows], [1, 1, 2, 2])
        self.assertAlmostEqual(rows[0][5], 0)
        self.assertAlmostEqual(rows[2][5], 1 / 104)
        self.assertEqual(reader.samples(), [])

    def test_wrap_and_lost_records(self):
        memory = SimulatedMemory()
        reader = BufferReader(memory, 0)
        memory.advance(0.25)  # 52 项，尚未回绕。
        self.assertEqual(len(reader.samples()), 52)
        memory.advance(0.4)  # 跨缓冲区边界读取。
        rows = reader.samples()
        self.assertEqual([row[0] for row in rows], list(range(53, 83)))
        memory.advance(1.0)  # 主机故意落后，只保留最近 63 个已提交槽位。
        rows = reader.samples()
        self.assertEqual(len(rows), 63)
        self.assertEqual(reader.host_dropped, 63)
        self.assertEqual(rows[-1][0], 208)

    def test_overwrite_during_read_is_retried(self):
        class ConcurrentMemory(SimulatedMemory):
            overwrite = True

            def read(self, address, size):
                raw = super().read(address, size)
                if address >= HEADER.size and self.overwrite:
                    self.overwrite = False
                    self.advance(1.0)
                return raw

        memory = ConcurrentMemory()
        reader = BufferReader(memory, 0)
        memory.advance(0.02)
        self.assertEqual(reader.samples(), [])
        self.assertEqual(reader.retries, 1)
        self.assertEqual(len(reader.samples()), 63)
        self.assertEqual(reader.host_dropped, 145)

    def test_incomplete_slot_not_published(self):
        memory = SimulatedMemory()
        reader = BufferReader(memory, 0)
        memory.advance(0.01)
        memory.records[0:4] = b"\0" * 4
        self.assertEqual(reader.samples(), [])
        self.assertEqual(reader.cursor, 0)

    def test_status_guard_and_reset(self):
        memory = SimulatedMemory()
        reader = BufferReader(memory, 0)
        memory.header[5] = 1
        self.assertIsNone(reader.state())
        memory.header[5] = 2
        self.assertEqual(reader.state()["who_am_i"], 0x6B)
        memory.advance(0.02)
        reader.samples()
        memory.header[4] = 0
        with self.assertRaisesRegex(RuntimeError, "复位"):
            reader.samples()

    def test_header_error_reports_actual_values(self):
        memory = SimulatedMemory()
        memory.header[0] = 0x2000009C
        with self.assertRaisesRegex(RuntimeError, "读回：magic=0x2000009C"):
            BufferReader(memory, 0)

    def test_flash_check_includes_ram_initializers(self):
        elf = Path(__file__).resolve().parents[2] / "STM32_F103/build/Debug/STM32_F103.elf"
        if not elf.exists():
            self.skipTest("ARM 固件尚未构建")
        segments = flash_segments(elf)

        class FlashMemory:
            def __init__(self):
                self.data = bytearray(b"\xff" * 65536)
                for address, raw in segments:
                    start = address - 0x08000000
                    self.data[start:start + len(raw)] = raw

            def read(self, address, size):
                start = address - 0x08000000
                return self.data[start:start + size]

        memory = FlashMemory()
        self.assertEqual(verify_firmware(memory, elf), sum(len(raw) for _, raw in segments))
        # Ozone 可不写节间对齐空洞；此处保持擦除态必须通过，不能误报旧固件。
        self.assertEqual(memory.data[0x10C:0x110], b"\xff" * 4)
        self.assertEqual(verify_firmware(memory, elf), sum(len(raw) for _, raw in segments))
        # 特意只破坏 RAM 初值镜像中的日志 magic，向量表和代码保持一致。
        log_magic = b"LUMI\x01\0\0\0\x40\0\0\0\x24\0\0\0"
        address, raw = next((address, raw) for address, raw in segments if log_magic in raw)
        offset = address - 0x08000000 + raw.index(log_magic)
        memory.data[offset] ^= 0xFF
        with self.assertRaisesRegex(RuntimeError, "目标 Flash 与 ELF 不一致"):
            verify_firmware(memory, elf)

    def test_summary_statistics(self):
        stats = Statistics()
        stats.add((1, 2, 3))
        stats.add((3, 4, 5))
        self.assertEqual(stats.result()["axes_and_norm_mean"][:3], [2, 3, 4])
        self.assertAlmostEqual(stats.result()["axes_and_norm_std"][0], 2 ** 0.5)

    def test_csv_and_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            args = argparse.Namespace(duration=0.12, poll_ms=5)
            capture(SimulatedMemory(), 0, args, output, {"mode": "simulation"})
            with (output / "samples.csv").open(encoding="utf-8", newline="") as file:
                rows = list(csv.DictReader(file))
            self.assertGreaterEqual(len(rows), 10)
            self.assertEqual(rows[0]["sensor"], "accel")
            self.assertEqual(rows[1]["unit"], "rad/s")
            summary = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(summary["mode"], "simulation")
            self.assertEqual(summary["host_dropped"], 0)

    def test_actual_elf_symbol_when_built(self):
        elf = Path(__file__).resolve().parents[2] / "STM32_F103/build/Debug/STM32_F103.elf"
        if not elf.exists():
            self.skipTest("ARM 固件尚未构建")
        try:
            address = find_symbol(elf)
        except RuntimeError as exc:
            if "找不到 arm-none-eabi-nm" in str(exc):
                self.skipTest(str(exc))
            raise
        self.assertGreaterEqual(address, 0x20000000)
        self.assertLess(address, 0x20005000)


if __name__ == "__main__":
    unittest.main()
