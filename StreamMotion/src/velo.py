# Function to create the value of velocity for either joint or cartesian given the value of m
from src.radius import *

def velocity(m):

    if (m == 0):    # Cartesian Velocity
        v = int(input("Input a value for maximum velocity in mm/s (0 < v < 255): "))
        axis = int(input("Input an axis for motion (1 = X, 2 = Y, 3 = Z): "))
        v = v*8/1000

    elif (m == 1):  # Joint Velocity

        while(True):
            velocity = int(input("Input a value for Max Velocity in deg/s: "))
            # selects joint for movement, joint-1 is the index for the joint within jnt_data 
            axis = int(input("Input a joint(1-6) for rotation: "))

            # Gets radius of the joint based on the initial position of the arm
            r = radius(axis)

            # Converts deg/s into rad/s 
            v_rads = math.radians(velocity)
            # Converst rad/s into mm/s
            v_mm = v_rads*r

            if (v_mm < 255):
                v = velocity*8/1000
                break
            else:
                print("Velocity is too high!!")
        
        print("----------------------------")

    return v, axis
