#!/usr/bin/env python3

from cloversim.randomization import create_random_string, randfloat, randbool, create2d_postions

marker_positions_raw = create2d_postions([
    .5,
    .5,
    .5,
], (3, 3), (0.5, 0.5))

MARKER_POSITIONS = {
    "red": marker_positions_raw[0],
    "green": marker_positions_raw[1],
    "blue": marker_positions_raw[2],
}