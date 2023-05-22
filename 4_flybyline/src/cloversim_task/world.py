#!/usr/bin/env python3

from cloversim.generation import World, Include, Box
from cloversim.generation import ImageTextures
from .randomization import *
from os import path
import math

assets_path = path.join(path.dirname(__file__), 'assets')

image_textures = ImageTextures({
    "tile_s": path.join(assets_path, "tile_s.png"),
    "tile_l": path.join(assets_path, "tile_l.png"),
    "tile_r": path.join(assets_path, "tile_r.png"),
    "tile_e": path.join(assets_path, "tile_e.png"),
})
image_textures.generate_materials()

WORLD = World()
WORLD.add(Include("model://sun"))
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

tile_map = [
    image_textures["tile_s"], image_textures["tile_r"],
    image_textures["tile_l"]
]

last_tile = PATH[-1]

for i, (cords, tile) in enumerate(TILES.items()):
  rot = tile[0] * math.pi / 2
  texture = tile_map[tile[1]]
  if cords == last_tile:
    texture = image_textures["tile_e"]

  WORLD.add(
      Box(f"tile_{i}",
          size=(TILE_SIZE, TILE_SIZE, 0.001),
          pose=(cords[0] * TILE_SIZE, cords[1] * TILE_SIZE, 0.001, 0, 0, rot),
          mass=0.1,
          material=texture,
          static=True))
