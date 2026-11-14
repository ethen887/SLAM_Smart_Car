"""Convert the RA8P1 g_build_map_cells debugger memory dump to a PGM image."""
import argparse
import struct
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("dump", type=Path)
parser.add_argument("output", type=Path)
args = parser.parse_args()
data = args.dump.read_bytes()
cell = struct.Struct("<fIB3x")
if len(data) != 100 * 100 * cell.size:
    raise SystemExit(f"Expected {100 * 100 * cell.size} bytes, got {len(data)}; check Cell layout and dump bounds")
pixels = bytearray()
for y in reversed(range(100)):
    for x in range(100):
        log_odds, _, observed = cell.unpack_from(data, (y * 100 + x) * cell.size)
        pixels.append(127 if not observed else (0 if log_odds > 0 else 255))
args.output.write_bytes(b"P5\n100 100\n255\n" + pixels)
print(f"Saved {args.output}: black=occupied, white=free, gray=unknown")
