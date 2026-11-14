import math
import struct
import unittest
from real_mapping import StreamParser, crc16


def packet(seq=1):
    payload = struct.pack('<IfffBffff', seq, 0.5, -0.25, 0.1, 2, 0.0, 1.0, 1.57, 2.0)
    body = struct.pack('<H', len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


class RealStreamTest(unittest.TestCase):
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
