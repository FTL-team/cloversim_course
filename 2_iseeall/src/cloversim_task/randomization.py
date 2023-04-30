#!/usr/bin/env python3

# Write your randomization code here
from cloversim.randomization import create_random_string
import random

qrcode_contents = create_random_string(32)
land_color = random.choice(["red", "green", "blue"])
