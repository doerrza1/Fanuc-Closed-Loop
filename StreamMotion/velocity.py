# Stream Motion test for controlling a steady state velocity, using z - axis motion

from src.utils import *
from src.client import *
import numpy as np

# 8ms per signal, 125 signals per second
# velocity = 125 mm/s = .125 mm/ms * 8 = 1 mm/8ms

v = int(input("Input a value for maximum velocity in mm/s (0 < v < 255): "))

# conversion from mm/s to mm/ms * 8
v_ms = v*8/1000


