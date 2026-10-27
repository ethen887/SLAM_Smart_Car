"""python -m unittest discover -s tools/odometry -p 'test_*.py' -v，无硬件运动。"""
import argparse
from contextlib import redirect_stdout, redirect_stderr
import io
import json
import math
import struct
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from unittest.mock import Mock

import run_straight as run


class Clock:
    def __init__(self):
        self.t = 100.0

    def monotonic(self):
        return self.t

    def sleep(self, seconds):
        self.t += seconds


class RunTests(unittest.TestCase):
    def test_layout(self):
        self.assertEqual(run.HEADER.size, 32)
        self.assertEqual(run.COMMAND.size, 32)
        self.assertEqual(run.RECORD.size, 112)
        self.assertEqual(len(run.FIELDS), 26)

    def test_partial_hardware_write_is_error(self):
        memory = run.HardwareMemory.__new__(run.HardwareMemory)
        memory.link = Mock()
        memory.link.memory_write32.return_value = 0
        with self.assertRaisesRegex(RuntimeError, '不完整'):
            memory.write(0x20000000, run.U32.pack(2))

    def test_actual_ram_configuration_decode(self):
        symbols = {name: i*64 for i, name in enumerate((
            'odometry_config', 'test_run_config', 'odometry_counts_per_rev', 'left_motor_pid', 'right_motor_pid'))}
        class Memory:
            def read(self, address, size):
                if address == symbols['odometry_config']:
                    return struct.pack('<3f2i', 0.035, 0.036, 0.15, 1, -1)
                if address == symbols['test_run_config']:
                    return struct.pack('<4f2I', 2, 0.6, 0.03, 0.3, 500, 120000)
                if address == symbols['odometry_counts_per_rev']:
                    return struct.pack('<f', 60000)
                if address in (symbols['left_motor_pid'], symbols['right_motor_pid']):
                    return struct.pack('<I', 0x20001000)
                return struct.pack('<10fB3x', *range(1, 11), 1)
        config = run.configuration(Memory(), symbols)
        self.assertEqual(config['counts_per_wheel_rev'], 60000)
        self.assertEqual(config['odometry']['right_direction'], -1)
        self.assertAlmostEqual(config['odometry']['left_radius_m'], 0.035)
        self.assertEqual(config['left_pid']['ki'], 2)
        self.assertEqual(config['left_pid']['d_first'], 1)

    def test_firmware_error_not_overwritten_by_cleanup(self):
        class Failed(run.SimulatedRun):
            def advance(self, elapsed):
                super().advance(elapsed)
                if self.t > 0.5:
                    self.header[6:8] = [7, 5]

        clock = Clock()
        args = argparse.Namespace(speed=0.1, x=1.0, y=0.0, distance=0.1, stop_mode='time')
        meta = {'mode': 'simulation', 'distance_m': 0.1, 'stop_mode': 'time',
                'configuration': {'counts_per_wheel_rev': 60000, 'odometry': {
                    'left_radius_m': 0.03, 'right_radius_m': 0.03, 'left_direction': 1, 'right_direction': -1}}}
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(run.time, 'monotonic', clock.monotonic), \
             patch.object(run.time, 'sleep', clock.sleep):
            result = run.capture(Failed(), Failed.symbols, args, Path(directory), meta)
            self.assertEqual(result['firmware_error'], 'invalid_odometry')
            self.assertEqual(result['state'], 'error')

    def test_ring_order_wrap_and_loss(self):
        memory = run.SimulatedRun()
        reader = run.RunReader(memory, memory.symbols['test_run_log'])
        memory.advance(0.20)
        rows = reader.samples()
        self.assertEqual([r['ticket'] for r in rows], list(range(1, memory.header[4]+1)))
        self.assertEqual(reader.samples(), [])
        memory.advance(0.4)
        self.assertGreater(len(reader.samples()), 0)
        memory.advance(1.0)
        rows = reader.samples()
        self.assertEqual(len(rows), 31)
        self.assertGreater(reader.dropped, 0)
        self.assertEqual(rows[-1]['ticket'], memory.header[4])

    def test_partial_record_and_reset(self):
        memory = run.SimulatedRun()
        reader = run.RunReader(memory, memory.symbols['test_run_log'])
        memory.advance(0.05)
        memory.ring[:4] = bytes(4)
        self.assertEqual(reader.samples(), [])
        self.assertEqual(reader.retries, 1)
        memory.header[4] = 0
        with self.assertRaisesRegex(RuntimeError, '复位'):
            reader.samples()

    def test_command_commit_and_heartbeat_loss(self):
        memory = run.SimulatedRun()
        args = argparse.Namespace(speed=0.1, x=0, y=1, distance=0.02, stop_mode='time')
        guard = run.send_command(memory, memory.symbols['test_run_command'], 1, args)
        packet = run.COMMAND.unpack(memory.command)
        self.assertEqual(packet[0], guard)
        self.assertAlmostEqual(packet[4], math.pi/2, places=6)
        memory.advance(0.1)
        self.assertEqual(memory.header[5], guard)
        memory.advance(0.8)  # 不发送心跳，模拟器应中止。
        self.assertEqual(memory.header[6:8], [7, 2])

    def test_end_to_end_simulation_and_normalization(self):
        clock = Clock()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(run.time, 'monotonic', clock.monotonic), \
             patch.object(run.time, 'sleep', clock.sleep), redirect_stdout(io.StringIO()):
            output = Path(directory)/'run'
            code = run.main(['--simulate', '--speed', '0.1', '--x', '3', '--y', '4',
                             '--distance', '0.02', '--output', str(output)])
            self.assertEqual(code, 0)
            meta = json.loads((output/'metadata.json').read_text(encoding='utf-8'))
            summary = json.loads((output/'summary.json').read_text(encoding='utf-8'))
            self.assertEqual(meta['mode'], 'simulation')
            self.assertAlmostEqual(meta['direction_unit'][0], 0.6)
            self.assertAlmostEqual(meta['velocity_xy_mps'][1], 0.08)
            self.assertEqual(summary['state'], 'done')
            self.assertEqual(summary['motion_host_dropped'], 0)
            self.assertEqual(summary['imu_host_dropped'], 0)
            self.assertGreater(summary['imu_accel']['samples'], 0)
            self.assertIsNone(summary['actual_distance_m'])
            self.assertTrue((output/'motion.csv').exists())
            self.assertTrue((output/'imu_state.csv').exists())
            self.assertTrue((output/'imu_samples.csv').exists())

    def test_odometry_stop_mode_and_measured_distance(self):
        clock = Clock()
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(run.time, 'monotonic', clock.monotonic), \
             patch.object(run.time, 'sleep', clock.sleep), redirect_stdout(io.StringIO()):
            code = run.main(['--simulate', '--speed', '0.1', '--distance', '0.02', '--stop-mode',
                             'odometry', '--actual-distance', '0.025', '--output', directory+'/run'])
            self.assertEqual(code, 0)
            result = json.loads(Path(directory, 'run/summary.json').read_text(encoding='utf-8'))
            self.assertEqual(result['stop_mode'], 'odometry')
            self.assertEqual(result['actual_distance_m'], 0.025)
            self.assertGreater(result['radius_scale_suggestion'], 1)

    def test_interrupt_requests_stop_and_keeps_log(self):
        class Interrupted(run.SimulatedRun):
            interrupted = False

            def read(self, address, size):
                if self.t > 0.5 and not self.interrupted:
                    self.interrupted = True
                    raise KeyboardInterrupt
                return super().read(address, size)

        clock = Clock()
        memory = Interrupted()
        args = argparse.Namespace(speed=0.1, x=1.0, y=0.0, distance=0.1, stop_mode='time')
        meta = {'mode': 'simulation', 'distance_m': 0.1, 'stop_mode': 'time',
                'configuration': {'counts_per_wheel_rev': 60000, 'odometry': {
                    'left_radius_m': 0.03, 'right_radius_m': 0.03, 'left_direction': 1, 'right_direction': -1}}}
        with tempfile.TemporaryDirectory() as directory, \
             patch.object(run.time, 'monotonic', clock.monotonic), \
             patch.object(run.time, 'sleep', clock.sleep):
            result = run.capture(memory, memory.symbols, args, Path(directory), meta)
            self.assertEqual(memory.header[6], 6)
            self.assertEqual(result['state'], 'aborted')
            self.assertIn('Ctrl+C', result['error'])
            self.assertTrue(Path(directory, 'summary.json').exists())

    def test_release_and_reject_invalid_inputs(self):
        clock = Clock()
        with patch.object(run.time, 'monotonic', clock.monotonic), \
             patch.object(run.time, 'sleep', clock.sleep), \
             redirect_stdout(io.StringIO()), redirect_stderr(io.StringIO()):
            self.assertEqual(run.main(['--simulate', '--release']), 0)
            for args in (['--speed', '0'], ['--x', '0', '--y', '0'], ['--speed', '1'],
                         ['--distance', 'nan'], ['--actual-distance', '-1']):
                self.assertEqual(run.main(['--simulate', *args]), 1)


if __name__ == '__main__':
    unittest.main()
