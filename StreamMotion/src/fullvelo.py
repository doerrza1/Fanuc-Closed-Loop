# Function to obtain target velocities for all joints/axes, obtains acceleration times for all

from src.radius import *

# Pass in the value for type of motion (0 = Cartesian, 1 = Joint)

def motion(m):
    v = []
    i = 0
    # Cartesian Motion
    if (m == 0):
        while (i < 3):
            # X-Axis
            if (i == 0):
                confirm = input("X-Axis Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    # velo(motion_type, axis)
                    a = velo(m, 0)
                else:
                    a = [0, 0, 0]
            # Y-Axis
            elif (i == 1):
                confirm = input("Y-Axis Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m, 1)

                else:
                    a = [0, 0, 0]
            # Z-Axis
            elif (i == 2):
                confirm = input("Z-Axis Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m, 2)

                else:
                    a = [0, 0, 0]
            # Appends the lists obtained from A into 
            v.append(a)
            # Increments the while loop variable
            i += 1

    # Joint Motion
    elif(m == 1):
        while(i < 6):
            # Joint 1
            if (i == 0):
                confirm = input("Joint 1 Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m, 1)
                else:
                    a = [0, 0, 0]
            
            # Joint 2
            elif (i == 1):
                confirm = input("Joint 2 Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m, 2)
                else:
                    a = [0, 0, 0]
            
            # Joint 3
            elif (i == 2):
                confirm = input("Joint 3 Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m, 3)
                else:
                    a = [0, 0, 0]
            
            # Joint 4
            elif (i == 3):
                confirm = input("Joint 4 Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m, 4)
                else:
                    a = [0, 0, 0]
            
            # Joint 5
            elif (i == 4):
                confirm = input("Joint 5 Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m,5)
                else:
                    a = [0, 0, 0]
            
            # Joint 6
            elif (i == 5):
                confirm = input("Joint 6 Motion (y/n): ").strip().upper()
                if (confirm == "Y"):
                    a = velo(m,6)
                else:
                    a = [0, 0, 0]
            
            # Append list to return
            v.append(a)
            # Increment while loop variable
            i += 1
        
    return v



# Returns the values of target velocity, and signal times within a list
def velo(m, axis):
    
    if (m == 0):
        
        # Obtains velocity value in mm/s
        v = int(input("Input a value for maximum velocity in mm/s (0 < v < 255): "))
        # Converts velocity value to mm/8ms
        v = v*8/1000
        # Obtains acceleration/deceleration time in seconds
        a_time = int(input("Input a time for accelertion/deceleration in seconds: "))
        # Obtains steady state time in seconds
        s_time = int(input("Input a time for steady state motion in seconds: "))
        # Converts time into number of signals, needs to be integer for signal definition
        a_time = int(a_time*1000/8)
        s_time = int(s_time*1000/8)

        print("----------------------------")
        a = [v, a_time, s_time]

    elif(m == 1):

        # Obtains the value for angular velocity, checks for velocity that exceeds max for joint
        while(True):
            
            velocity = int(input("Input a value for Max Velocity in deg/s: "))

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
        
        #Obtains times for motion
        a_time = int(input("Input a time for accelertion/deceleration in seconds: "))
        s_time = int(input("Input a time for steady state motion in seconds: "))
        
        # Converts time into number of signals, needs to be integer for signal definition
        a_time = int(a_time*1000/8)
        s_time = int(s_time*1000/8)

        print("----------------------------")
        a = [v, a_time, s_time]

    # Returns list in form [velocity, accel/deccel, steady state]
    return a

        