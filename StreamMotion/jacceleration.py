# Stream Motion test for controlling acceleration/deceleration to a given velocity for joint motion
# J3 and J2 work in tandem with one another do not increment both

from src.client import *
from src.utils import *
from src.display import *
from src.radius import *
import numpy as np


acceleration = int(input("Input an acceleration value in deg/s^2: "))

# Grabs and validates target velocity
velocity = 1
while(velocity == True):
    velocity = int(input("Input a value for target velocity in deg/s: "))
    # selects joint for movement, joint-1 is the index for the joint within jnt_data 
    joint = int(input("Input a joint(1-6) for rotation: "))

    # Gets radius of the joint based on the initial position of the arm
    r = radius(joint)

    # Converts deg/s into rad/s 
    v_rads = math.radians(velocity)
    # Converst rad/s into mm/s
    v_mm = v_rads*r

    if (v_mm < 255):
        break
    else:
        print("Velocity is too high!!")
    
    print("----------------------------")


#Sets the velocity per every 8ms
v_ms = velocity*8/1000

acc_array = np.linspace()


