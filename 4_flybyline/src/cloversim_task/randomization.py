#!/usr/bin/env python3

# Write your randomization code here
import cloversim.randomization
import random

TILE_SIZE = 0.8


def generate_line():
  directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]
  cur_direction = 0

  cur_cords = (1, 0)
  path = [(0, 0)]
  tiles = {(0, 0): (0, 0)}

  direction_choose = [-1, 0, 1]
  for i in range(40):
    random.shuffle(direction_choose)
    found = False
    for next_direction_d in direction_choose:
      next_direction = (cur_direction + next_direction_d) % 4
      next_cords = (cur_cords[0] + directions[next_direction][0],
                    cur_cords[1] + directions[next_direction][1])
      if next_cords in tiles:
        continue
      if abs(next_cords[0]) > 8 or abs(next_cords[1]) > 8:
        continue

      path.append(cur_cords)
      tiles[cur_cords] = (cur_direction, next_direction_d)
      found = True

      cur_cords = next_cords
      cur_direction = next_direction
      break

    if not found:
      return None

  return (tiles, path)


LINE = None
while LINE is None:
  LINE = generate_line()

TILES, PATH = LINE
