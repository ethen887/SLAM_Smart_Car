import struct
import tkinter as tk
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from real_mapping import (NAV_PATH_RECEIVED, NAV_PLAN, NAV_PREVIEW, NAV_READY,
                          RealMappingWindow, StreamParser, crc16, navigation_command)


def path_frame(seq, points):
    payload = struct.pack('<BBBBH', ord('P'), 1, seq, 0, len(points))
    payload += b''.join(struct.pack('<ff', *point) for point in points)
    body = struct.pack('<H', len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class NavigationUITest(unittest.TestCase):
    def test_split_path_and_bad_crc(self):
        points = ((0, 0), (0.25, 0.5))
        frame = path_frame(42, points)
        for cut in range(1, len(frame)):
            parser = StreamParser()
            self.assertEqual(parser.feed(frame[:cut]), [])
            self.assertEqual(parser.feed(frame[cut:]), [('path', 42, points)])
        damaged = frame[:-1] + bytes([frame[-1] ^ 1])
        parser = StreamParser()
        self.assertEqual(parser.feed(damaged + frame), [('path', 42, points)])
        self.assertEqual(parser.feed(path_frame(0, ((float('nan'), 0),))), [])

    def test_map_goal_preview_and_board_status(self):
        root = tk.Tk()
        root.withdraw()
        window = RealMappingWindow(root, 'unused.dll')
        try:
            root.update()
            device = window.device = Mock()
            device.write.side_effect = len
            click = SimpleNamespace(inaxes=window.ax, button=1, xdata=0.25, ydata=-0.5)
            window.select_goal(click)
            device.write.assert_not_called()
            window.map_pose = (0, 0, 0)
            window.select_goal(click)
            device.write.assert_called_once_with(navigation_command(NAV_PLAN, 0, (0.25, -0.5)))
            self.assertEqual(struct.unpack_from('<ff', device.write.call_args.args[0], 8), (0.25, -0.5))
            window.latest_path = ('path', 255, ((0, 0), (0.5, 0.5)))
            window.tick()
            self.assertEqual(len(window.path_line.get_xdata()), 0)
            self.assertEqual(device.write.call_count, 1)
            points = ((0, 0), (0.25, -0.5))
            window.latest_path = ('path', 0, points)
            window.queue.put(('navigation', NAV_PLAN, 0, NAV_PREVIEW, 0, 2, 0))
            window.tick()
            self.assertEqual(tuple(window.path_line.get_xdata()), (0, 0.25))
            self.assertEqual(tuple(window.goal_dot.get_ydata()), (-0.5,))
            self.assertEqual(device.write.call_args.args[0], navigation_command(NAV_PATH_RECEIVED, 0))
            window.queue.put(('navigation', NAV_PLAN, 0, NAV_READY, 0, 2, 0))
            window.tick()
            self.assertIn('按手柄 START 发车', window.nav_status.get())
            device.write.reset_mock()
            window.disconnect()
            device.close.assert_called_once()
            device.write.assert_not_called()
        finally:
            root.destroy()
