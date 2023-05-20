#!/usr/bin/env python3

# Write your checker there

from .randomization import *
from cloversim.utils import distance_between_points
from cloversim.score import ScoreTask, Scoring
from cloversim.checker import get_clover_position, is_clover_armed, is_clover_still
import rospy
import math

landing = ScoreTask('Landed', 20)
on_platform = ScoreTask('Above platform', 30)

scoring = Scoring('Land in the middle', [landing, on_platform])

current_point = 0
last_land_position = (0, 0, 0)

def process_position():
  global current_point
  global last_land_position

  pos = get_clover_position()
  is_still = is_clover_still(pos)
  armed = is_clover_armed()

  position = pos.pose.position
  x, y, z = position.x, position.y, position.z

  if is_still and not armed:
    landing.set_score(20)
  else:
    landing.set_score(0)

  if is_still:
    dist = math.sqrt((x - LAND_POS[0]) ** 2 + (y - LAND_POS[1]) ** 2)
    if dist < 0.4:
      on_platform.set_score(30)
    else:
      on_platform.set_score(0)

while True:
  try:
    process_position()
  except Exception as E:
    print(E)
    pass
  rospy.sleep(0.2)