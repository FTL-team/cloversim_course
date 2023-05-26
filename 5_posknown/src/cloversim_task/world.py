#!/usr/bin/env python3

from .randomization import *
from cloversim.generation import World, Include
from cloversim.generation import ArucoMap, generate_aruco_map
from cloversim.generation import Box, ColorMaterial

WORLD = World()
WORLD.add(Include("model://sun"))
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

WORLD.add(
    ArucoMap("aruco_map",
             generate_aruco_map(number_of_markers=(5, 5))).generate())

color_map = {
    "red": ColorMaterial((1, 0, 0)),
    "green": ColorMaterial((0, 1, 0)),
    "blue": ColorMaterial((0, 0, 1)),
}

for i, color in enumerate(color_map.keys()):
  pose = MARKER_POSITIONS[color]
  WORLD.add(
      Box("marker_" + color,
          size=(0.4, 0.4, 0.001),
          pose=(pose[0], pose[1], 0.001),
          material=color_map[color],
          static=True))
