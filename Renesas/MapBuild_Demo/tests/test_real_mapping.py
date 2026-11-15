import math
import struct
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
import numpy as np
from matplotlib.figure import Figure
from real_mapping import RealMappingWindow, StreamParser, crc16


def packet(seq=1):
    payload = struct.pack('<IfffBffff', seq, 0.5, -0.25, 0.1, 2, 0.0, 1.0, 1.57, 2.0)
    body = struct.pack('<H', len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class RealStreamTest(unittest.TestCase):
    def test_fixed_map_view_and_rotating_heading(self):
        grid = np.zeros((2, 2), dtype=[('seen', 'u1'), ('log', 'f4')])
        grid['seen'][0, 1] = 1
        grid['log'][0, 1] = 2
        window = SimpleNamespace(
            grid=SimpleNamespace(snapshot=lambda: grid,
                                 cfg=SimpleNamespace(origin_x=-1, origin_y=-1,
                                                     width=2, height=2, resolution=1)),
            ax=Figure().add_subplot(111), canvas=SimpleNamespace(draw_idle=Mock()))
        RealMappingWindow.draw(window, (0, 0, 0))
        image = np.array(window.ax.images[0].get_array())
        first_arrow = window.ax.patches[0].get_path().vertices.copy()
        RealMappingWindow.draw(window, (0, 0, math.pi / 2))
        np.testing.assert_array_equal(image, window.ax.images[0].get_array())
        self.assertEqual(window.ax.get_xlim(), (-1, 1))
        self.assertEqual(window.ax.get_ylim(), (-1, 1))
        self.assertFalse(np.allclose(first_arrow, window.ax.patches[0].get_path().vertices))

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
