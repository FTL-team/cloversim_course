#!/usr/bin/env python3

# Write your checker there

from cloversim.utils import distance_between_points
from .randomization import *
from cloversim.score import ScoreTask, Scoring
from cloversim.checker import get_clover_position, is_clover_armed, is_clover_still
import rospy

follow_line = ScoreTask('Follow Line', 80)
landing = ScoreTask('Land on finish', 10)

scoring = Scoring('Fly by line', [follow_line, landing])

current_point = 0
last_land_position = (0, 0, 0)

cur_path = PATH


def process_position():
  global cur_path

  pos = get_clover_position()
  is_still = is_clover_still(pos)
  armed = is_clover_armed()

  position = pos.pose.position
  x, y, z = position.x, position.y, position.z

  if is_still and not armed:
    cur_path = PATH[-2::-1]

  tile_x = round(x / TILE_SIZE)
  tile_y = round(y / TILE_SIZE)

  if len(cur_path) > 0 and cur_path[-1] == (tile_x, tile_y):
    cur_path.pop()
    follow_line.set_score(80 - len(cur_path) * 2)

  print(tile_x, tile_y, len(PATH), len(cur_path), cur_path[-3:])

  if is_still and not armed and (tile_x, tile_y) == PATH[-1]:
    landing.set_score(10)
  else:
    landing.set_score(0)


while True:
  try:
    process_position()
  except Exception as E:
    print(E)
    pass
  rospy.sleep(0.2)