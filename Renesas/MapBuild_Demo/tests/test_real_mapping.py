import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from matplotlib.figure import Figure
from real_mapping import RealMappingWindow, StreamParser, crc16, map_rays, relative_pose, real_map_settings, obstacle_boxes, navigation_command
from simulation import Grid, defaults
from run_demo import build


def packet(seq=1):
    payload = struct.pack('<IfffBffff', seq, 0.5, -0.25, 0.1, 2, 0.0, 1.0, 1.57, 2.0)
    body = struct.pack('<H', len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class RealStreamTest(unittest.TestCase):
    def test_navigation_status_interleaves_map_and_resynchronizes(self):
        payload = b'N\x01' + struct.pack('<BBBBHH', 3, 42, 7, 2, 123, 10)
        body = struct.pack('<H', len(payload)) + payload
        status = b'\xa5\x5a' + body + struct.pack('<H', crc16(body))
        bad = bytearray(status); bad[-1] ^= 1
        parser = StreamParser()
        self.assertEqual(parser.feed(status[:5]), [])
        rows = parser.feed(status[5:] + bytes(bad) + packet(99))
        self.assertEqual(rows[0], ('navigation', 3, 42, 7, 2, 123, 10))
        self.assertEqual(rows[1][0], 99)
        self.assertGreater(parser.bad, 0)

    def test_command_write_requires_board_confirmation(self):
        frame = navigation_command(4, 255)
        self.assertEqual(frame[:8], b'\xa5\x5a\x04\x00N\x01\x04\xff')
        self.assertEqual(crc16(frame[2:-2]), int.from_bytes(frame[-2:], 'little'))
        window = SimpleNamespace(device=Mock(), nav_sequence=255, nav_status=Mock())
        window.device.write.return_value = 10
        RealMappingWindow.send_navigation(window, 4)
        self.assertEqual(window.nav_sequence, 0)
        self.assertEqual(window.pending_navigation[:2], (4, 255))
        window.device.write.assert_called_once_with(frame)

    def test_real_map_covers_six_meters(self):
        cfg = real_map_settings()
        self.assertAlmostEqual(cfg['width'] * cfg['resolution'], 6.0)
        self.assertAlmostEqual(cfg['height'] * cfg['resolution'], 6.0)
        self.assertEqual((cfg['origin_x'], cfg['origin_y']), (-3.0, -3.0))

    def test_first_pose_centers_map_without_rotating_frame(self):
        origin = (3.0, -5.0)
        self.assertEqual(relative_pose((3.0, -5.0, 1.2), origin), (0.0, 0.0, 1.2))
        self.assertEqual(relative_pose((4.0, -3.0, -2.0), origin), (1.0, 2.0, -2.0))

    def test_clockwise_lidar_wall_stays_fixed_during_turn(self):
        lib = build().with_name('libmapreal.dll' if __import__('sys').platform == 'win32' else 'libmapreal.so')
        grid = Grid(lib, defaults()['map'])
        try:
            # A wall point at (1, 0): clockwise raw angle advances as the car turns left.
            for frame, yaw in enumerate((0, math.pi / 2, -math.pi / 2), 1):
                rays = map_rays([(yaw, 1.0, 1)], reverse=True)
                grid.update((0, 0, yaw), rays, frame)
            cells = grid.snapshot()
            self.assertGreater(cells['log'][50, 70], 2.0)
            self.assertFalse(cells['seen'][50, 30])
        finally:
            grid.close()

    def test_fixed_map_view_and_rotating_heading(self):
        cells = np.zeros((100, 100), dtype=np.uint8)
        cells[0, 1] = 2
        window = SimpleNamespace(
            cells=cells,
            ax=Figure().add_subplot(111), canvas=SimpleNamespace(draw_idle=Mock()))
        RealMappingWindow.draw(window, (0, 0, 0))
        image = np.array(window.ax.images[0].get_array())
        self.assertEqual(window.ax.images[0].get_cmap().name, 'gray')
        self.assertLess(image[0, 1], image[0, 0])  # Occupied has lower brightness.
        self.assertLess(window.ax.images[0].cmap(image[0, 1])[0],
                        window.ax.images[0].cmap(image[0, 0])[0])  # And is darker.
        first_arrow = window.ax.patches[-1].get_path().vertices.copy()
        RealMappingWindow.draw(window, (0, 0, math.pi / 2))
        np.testing.assert_array_equal(image, window.ax.images[0].get_array())
        self.assertEqual(window.ax.get_xlim(), (-3, 3))
        self.assertEqual(window.ax.get_ylim(), (-3, 3))
        self.assertFalse(np.allclose(first_arrow, window.ax.patches[-1].get_path().vertices))

    def test_snapshot_fragmented_and_reconnect(self):
        cells = np.zeros(10000, dtype=np.uint8)
        cells[0:4] = [0, 1, 2, 3]
        packed = (cells.reshape(-1, 4) * np.array([1, 4, 16, 64], dtype=np.uint8)).sum(axis=1).astype(np.uint8)
        payload = b'M\x03' + struct.pack('<IfffB', 1234, 0.1, -0.2, 0.3, 1) + packed.tobytes()
        body = struct.pack('<H', len(payload)) + payload
        wire = b'\xa5\x5a' + body + struct.pack('<H', crc16(body))
        parser = StreamParser()
        self.assertEqual(parser.feed(wire[:1000]), [])
        row, = parser.feed(wire[1000:])
        self.assertEqual(row[:2], ('map', 1234))
        np.testing.assert_array_equal(row[3].ravel()[:4], [0, 1, 2, 3])
        self.assertEqual(StreamParser().feed(wire)[0][1], 1234)

    def test_obstacle_boxes_use_closed_cells(self):
        mask = np.zeros((8, 8), dtype=bool)
        mask[1, 1:4] = True; mask[5:7, 6] = True
        self.assertEqual(obstacle_boxes(mask), [(1, 1, 4, 2), (6, 5, 7, 7)])

    def test_fragmented_and_coalesced(self):
        parser = StreamParser(); frame = packet()
        self.assertEqual(parser.feed(frame[:11]), [])
        rows = parser.feed(frame[11:] + packet(2))
        self.assertEqual([r[0] for r in rows], [1, 2])
        self.assertEqual(rows[0][1][:2], (0.5, -0.25))
        self.assertTrue(math.isclose(rows[0][2][1][0], 1.57, abs_tol=1e-6))

    def test_bad_crc_resync(self):
        bad = bytearray(packet()); bad[10] ^= 1
        parser = StreamParser()
        rows = parser.feed(b'junk' + bad + packet(3))
        self.assertEqual([r[0] for r in rows], [3])
        self.assertGreater(parser.bad, 0)


if __name__ == '__main__': unittest.main()
