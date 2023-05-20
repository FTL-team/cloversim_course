#!/usr/bin/env python3

# Write your randomization code here
from cloversim.randomization import randfloat

LAND_POS = (0, 0)
while abs(LAND_POS[0]) < 0.7 or abs(LAND_POS[1]) < 0.7:
  LAND_POS = (randfloat(-3, 3), randfloat(-3, 3)) 
