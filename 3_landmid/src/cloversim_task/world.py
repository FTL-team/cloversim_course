#!/usr/bin/env python3

from .randomization import *
from cloversim.generation import World, Include, Box, Cylinder
from cloversim.generation import ColorMaterial

WORLD = World()
WORLD.add(Include("model://sun"))
WORLD.add(Include("model://parquet_plane", pose=(0, 0, -0.01)))

WORLD.add(
    Cylinder("landing_pad",
             radius=0.4,
             length=0.3,
             pose=(LAND_POS[0], LAND_POS[1], 0.15),
             material=(
                 ColorMaterial((0.5, 0.5, 0.5)),
                 ColorMaterial((1, 0.5, 0)),
                 ColorMaterial((1, 1, 0)),
             )))