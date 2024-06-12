# Calculate the radius of a given joint movement given the initial positions
# Works for J1 through J3 at the moment


import math

def radius(joint, data = [0, 350, 300]):

    # Variable declaration
    x = data[0]
    y = data[1]
    z = data[2]

    if (joint == 1):
        # Rotation about J1 will have a radius equal to the y-distance
        r = y

    elif(joint == 2):
        # Rotation about J2 will have a radius equal to the hypotenuse of y and z
        sum_sq = y**2 + z**2  # c^2 = a^2 + b^2
        r = math.sqrt(sum_sq) # c = sqrt(a^2 + b^2)

    elif(joint == 3):
        # Rotation about J3 will have a radius equal to the y- distance
        r = y

    elif(joint == 4):
        # Rotation about J4 will have a radius equal to the length of the eoat
        r = 0

    elif(joint == 5):
        # Rotation about J5 will have a radius equal to the length of the eoat
        r = 0
    elif(joint == 6):
        # Rotation about J6 will have a radius equal to the length of the eoat
        r = 0
    else:
        r = 0

    return r
    
    


