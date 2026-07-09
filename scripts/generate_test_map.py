#!/usr/bin/env python3
"""Generate a test PGM map (P5 binary) for the lattice planner example.

Creates a 200x200 pixel map (10m x 10m at 0.05m resolution) with:
  - Free space (white = 0)
  - A vertical wall at x=5m with a gap (forces planning around it)
  - Two small obstacle blocks

Output: examples/data/test_map.pgm
"""

import os
import struct

WIDTH = 200
HEIGHT = 200

def main():
    # Initialize all pixels to free (0 = black in PGM, but we invert: 0=free, 255=occupied)
    # Standard PGM: 0=black, 255=white. For maps: white=free, black=occupied.
    # We use: 255=free, 0=occupied (inverted grayscale, common in robotics).
    pixels = [[255] * WIDTH for _ in range(HEIGHT)]

    def set_rect(cx, cy, sx, sy, val=0):
        """Set a rectangle of pixels to val. cx,cy in meters, sx,sy in meters."""
        x0 = int((cx - sx / 2) / 0.05)
        x1 = int((cx + sx / 2) / 0.05)
        y0 = int((cy - sy / 2) / 0.05)
        y1 = int((cy + sy / 2) / 0.05)
        for y in range(max(0, y0), min(HEIGHT, y1 + 1)):
            for x in range(max(0, x0), min(WIDTH, x1 + 1)):
                pixels[y][x] = val

    # Vertical wall at x=5m, y=2-4m (gap from y=4-6m)
    set_rect(5.0, 3.0, 0.2, 2.0)
    # Vertical wall at x=5m, y=6-8m
    set_rect(5.0, 7.0, 0.2, 2.0)
    # Small obstacle block
    set_rect(3.0, 6.0, 0.5, 0.5)
    # Another obstacle
    set_rect(7.0, 2.0, 0.5, 0.5)

    # Write P5 PGM (binary)
    out_dir = os.path.join(os.path.dirname(__file__), '..', 'examples', 'data')
    out_path = os.path.join(out_dir, 'test_map.pgm')
    os.makedirs(out_dir, exist_ok=True)

    with open(out_path, 'wb') as f:
        header = f"P5\n{WIDTH} {HEIGHT}\n255\n"
        f.write(header.encode('ascii'))
        # PGM rows are top-to-bottom
        for row in range(HEIGHT):
            row_data = bytes(pixels[HEIGHT - 1 - row])  # flip to bottom-to-top
            f.write(row_data)

    print(f"Generated test map: {out_path} ({WIDTH}x{HEIGHT})")

if __name__ == '__main__':
    main()
