# Stream Motion test for controlling a steady state velocity wih joint motion
# J3 and J2 work in tandem with one another

from src.client import *
from src.utils import *
from src.display import *
from src.radius import *
import numpy as np

# Velocity input will be in deg/s, converted to deg/8ms (for signal definition)

# Will also convert the deg/s into mm/s using a calculated radius for the motion to 
# ensure that the input velocity is less than 255 mm/s

velocity = int(input("Input a value for Max Velocity in deg/s: "))
joint = int(input("Input a joint(1-6) for rotation: "))


               


