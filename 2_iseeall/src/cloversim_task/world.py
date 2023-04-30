#!/usr/bin/env python3

from .randomization import *
from cloversim.generation import World, Include, Box
from cloversim.generation import ColorMaterial, ImageTextures
import qrcode

WORLD = World()
WORLD.add(Include("model://sun"))

color_map = {"red": (1, 0, 0), "green": (0, 1, 0), "blue": (0, 0, 1)}

WORLD.add(
    Box("land",
        size=(10, 10, 0.08),
        mass=1,
        pose=(0, 0, -1),
        material=ColorMaterial(color_map[land_color]),
        static=True))

qrcode_texture = qrcode.make(qrcode_contents).get_image()
image_textures = ImageTextures({
    "qrcode": qrcode_texture,
})
image_textures.generate_materials()

WORLD.add(
    Box("qrcode",
        size=(1, 1, 0.001),
        pose=(0, 0, -0.9),
        mass=0.1,
        material=image_textures["qrcode"],
        static=True))

WORLD.add(
    Box("holder",
        size=(2, 2, 0.08),
        mass=1,
        pose=(0, 0, 0),
        material=ColorMaterial((0, 0, 0, 0)),
        static=True))
