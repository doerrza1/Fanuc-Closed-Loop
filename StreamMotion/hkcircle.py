# Fanuc Stream Motion Test Using (x-h)^2 + (y-k)^2 = r^2

from src.client import *
from src.utils import *
from src.display import *
import numpy as np

r = int(input("Enter a radius for the circle in mm: "))
sig = int(input("Enter the amount of signals: "))
