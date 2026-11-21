"""Convert the RA8P1 g_build_map_cells debugger memory dump to a PGM image."""
import argparse
import struct
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("dump", type=Path)
parser.add_argument("output", type=Path)
parser.add_argument('--width', type=int, default=300)
parser.add_argument('--height', type=int, default=300)
args = parser.parse_args()
if args.width <= 0 or args.height <= 0:
    parser.error('width and height must be positive')
width, height = args.width, args.height
data = args.dump.read_bytes()
cell = struct.Struct("<fIB3x")
if len(data) != width * height * cell.size:
    raise SystemExit(f"Expected {width * height * cell.size} bytes, got {len(data)}; check Cell layout and dump bounds")
pixels = bytearray()
for y in reversed(range(height)):
    for x in range(width):
        log_odds, _, observed = cell.unpack_from(data, (y * width + x) * cell.size)
        pixels.append(127 if not observed else (0 if log_odds > 0 else 255))
args.output.write_bytes(f"P5\n{width} {height}\n255\n".encode() + pixels)
print(f"Saved {args.output}: black=occupied, white=free, gray=unknown")
