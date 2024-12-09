import numpy as np
from src.plot import *
import serial
import math
import roboticstoolbox as rtb
import spatialmath as sm

# import scipy
import time

from src.client import *
from src.utils import *
from src.display import *
from src.limit import *

robot = rtb.models.URDF.id7l()
arduino_port = '/dev/ttyACM0'
baud_rate = 115200
timeout = 0.005
ser = serial.Serial(arduino_port, baud_rate, timeout=timeout)


# Encoder data to Position Data functions

def read_serial_data():
    try:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        if line:
            values = line.split(',')
            if len(values) == 3:
                # Split the line into position, velocity, and acceleration
                position, velocity, acceleration = values
                return float(position), float(velocity), float(acceleration)
            else:
                print("Not enough values received")

    except Exception as e:
        print(f"Error reading serial data: {e}")
    return None, None, None
    
def pause():
    programPause = input("Press the <ENTER> key to continue...")

def physicalParameters():
    g = 9.81 * np.sin(np.pi/6) # acceleration due to gravity (m/s^2)
    m = 0.1 # mass of the beam (kg)
    l = 0.5; # length of the beam (m)
    J = (1/12)*m*l**2 # mass moment of inertia of the beam about the center-of-mass (kg m^2)
    rp = 0.225 # distance of the center-of-mass from pinned joint (m)
    Jp = J + m*rp**2 # moment of inertia about the pin (kg m^2)
    M = 10 # mass of actuator (kg)
    
    return g, m, l, J, rp, Jp, M
    
def freeParameters():
    g, m, l, J, rp, Jp, M = physicalParameters()
    thk = np.pi/6 # the angle at which impulsive inputs are applied (rad)
    tha = np.pi/3 # the desired apex angle (rad)
    oms = - np.sqrt((2*m*g*rp/Jp)*(np.sin(tha) - np.sin(thk))) # steady-state pre-impact velocity (rad/s)
    # steady-state inputs
    rs  = 0.1
    Is  = -2*oms*Jp/(rp + rs)
    
    return thk, tha, oms, rs, Is
    
def actuatorParameters(omkm, rk, Ik, COR):
    g, m, l, J, rp, Jp, M = physicalParameters()
    thk, tha, oms, rs, Is = freeParameters()
    # define a parameter to use in calculations
    Pk = (rp + rk) + (Ik/omkm)*((1/M + (rp + rk)**2/Jp)/(1 + COR))
    # choose OM values
    OMX = 1
    OMY = 1
    # choose PSI values
    PSIX = np.pi/2
    PSIY = 0
    # solve for R
    RX = (Pk*np.sin(thk))/(OMX*np.sin(OMX*thk - PSIX))
    RY = (Pk*np.cos(thk))/(OMY*np.cos(OMY*thk - PSIY))
    # solve for R0
    RX0 = RX*np.cos(OMX*thk - PSIX) - (rp + rk)*np.cos(thk)
    RY0 = RY*np.sin(OMY*thk - PSIY) - (rp + rk)*np.sin(thk)
    
    return RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0    
    
def actVHC(th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0):
    # RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0 = actuatorParameters(omkm, rk, Ik, COR)
    # phi
    phiX = RX * np.cos(OMX * th - PSIX) - RX0
    phiY = RY * np.sin(OMY * th - PSIY) - RY0
    # d phi / d theta
    diffphiX = -RX * OMX * np.sin(OMX * th - PSIX)
    diffphiY = RY * OMY * np.cos(OMY * th - PSIY)
    # d2 phi / d theta2
    diff2phiX = -RX * OMX**2 * np.cos(OMX * th - PSIX)
    diff2phiY = -RY * OMY**2 * np.sin(OMY * th - PSIY)

    return phiX, phiY, diffphiX, diffphiY, diff2phiX, diff2phiY
    
def desiredPosVelAccn(th, dth, d2th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0):
    phiX, phiY, diffphiX, diffphiY, diff2phiX, diff2phiY = actVHC(th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
    X   = phiX
    Y   = phiY
    dX  = diffphiX*dth
    dY  = diffphiY*dth
    d2X = diff2phiX*dth**2 + diffphiX*d2th
    d2Y = diff2phiY*dth**2 + diffphiY*d2th

    return X, Y, dX, dY, d2X, d2Y
    
def zeroBeam():
    print("Place the beam at theta = 0 and maintain stationary")
    pause()
    tf = time.time() + 6 # time to record encoder data at theta = 0
    while time.time() < tf:
        th, dth, d2th = read_serial_data() # establishes the zero position
        last_th = th
        last_dth = dth
    return last_th, last_dth
    
def initialCondition():
    print(f"Place the beam at initial condition > {thk}")
    pause()
    tf = time.time() + 6 # time to record encoder data
    while time.time() < tf:
        th, dth, d2th = read_serial_data() # establishes the initial conditions
        print("Theta: ", th)
        last_th = th
        last_dth = dth
    try:
        omkm = - np.sqrt(last_dth**2 + (2*m*g*rp/Jp)*(np.sin(last_th) - np.sin(thk))) # initial pre-impact velocity
        return last_dth, omkm
    except:
        print("the initial condition is invalid -- go again")
    return last_dth, None

def isnum(num):
    try:
        float(num)
        return True
    
    except:
        return False

# Function to remedy any invalid signals from encoder (I don't believe this actually every gets called) 
def invalid_sig(X, Y, dX, dY, d2X, d2Y, dt = 0.008, g = 9.81):
    
    # new velocity = velocity + acceleration * change in time 
    new_dX = dX + d2X * dt
    new_dY = dY + d2Y * dt

    # new position = position + new velocity * change in time
    X = X + new_dX * dt
    Y = Y + new_dY * dt

    return X, Y, new_dX, new_dY

# Inverse Kinematics Functions

# Finds an SE3 matrix using the XY point and end effector position
# SE3 matrix is then used along with ikine_LM as teh inverse kinematics solver
# to find the joint angles required to reach the position
# (ikine_LM is called in a loop within the main function)

# Inverse kinematics requires the Robotics Toolbox for Python plugin as well as the .xacro file
# in rtb.data to load in the robot to the script
# robot is loaded in at the beginning by the import statements to be a global variable for all functions

def joint_angles(xy_point, wpr_pos):
    
    # Obtain XYZ position
    X = 0
    Y = xy_point[0]
    Z = xy_point[1]

    # Obtain WPR position in radians
    W = np.radians(wpr_pos[0])
    P = np.radians(wpr_pos[1])
    R = np.radians(wpr_pos[2])

    T = sm.SE3(X, Y, Z) * sm.SE3.RPY([W,P,R], order='zyx') # Create SE3 transform for inverse kinematics

    return T # Return SE3 matrix 

# Function to convert the radian array obtained from inverse kinematics to degrees
def rad_to_degrees(rad_array):
    deg_list = [] # Initialize empty list for easy appending
    # starting_degrees = [90, -18.8, -10.2, 0, -63.8, -90] Can be used to ensure angles start from proper position

    for array in rad_array:

        deg = np.degrees(array) % 360 # Convert the radian array to degrees within 0 to 360
        
        deg_list.append(deg) # Append to list

    deg_array = np.array(deg_list) # Convert list to numpy array to keep functionality

    return deg_array

# Limit Functions

# Functions below are used after the limit lookup tables are obtained
# Limit lookup table function and interpolation function are within limit.py so this must be imported
# in order to obtain the limits


# Finds the derivative of any np.array used within the script
# Called from within the check_limits function to find the appropriate values of vel, acc, and jerk 
# for each time step for easy comparisons

def derivative_array(array, dt = 0.008):

    # np.diff finds the difference between subsequent rows of the array
    # allowing for efficient calculation for derivatives (pos -> vel -> acc -> jerk)

    d_array = np.diff(array, axis=0) / dt 

    # Pad the first row with zeros to maintain the shape of array
    zero_array = np.zeros((1, array.shape[1]))
    d_array = np.vstack([zero_array, d_array])
    
    return d_array

# Finds the end effector velocities up to a given time step
# Called from within the check_limits function to find the appropiate v_peak for limit interpolation
# According to Fanuc v_peak is the highest velocity within the last 5 seconds of motion
# Utilizes the robotics toolbox plugin to convert joint velocities to cartesian velocity

def obtain_velocities(degree_array, time_step):
    ee_velos = []
   
    velocity_degrees = derivative_array(degree_array) # Obtain velocity array
    
    for i in range(time_step + 1):
        
        q = np.radians(degree_array[i]) # Convert position at time step to rad
        qd = np.radians(velocity_degrees[i]) # Convert velocity at time step to rad/s
        
        J = robot.jacob0(q) # Obtain jacobian matrix at time step

        ee_velo = J @ qd # Calculate end effector velocity, returns velocity in x, y, z directions
        linear_velo = np.sqrt(ee_velo[0]**2 + ee_velo[1]**2 + ee_velo[2]**2) * 1000 # Convert velocity vector to magnitude in (mm/s)
        ee_velos.append(linear_velo) 

    return ee_velos

# Main function for checking limits 
# Loops through each time step of the array to find the limits at that time step

# Uses np.greater to compare all joint values at one time step in a single line, this 
# returns a boolean array that is checked through np.any() to tell if any limit was exceeded
# If a limit is exceed it will be added to both a list and dictionary for future use

def check_limits(degrees_array, limit_matrices, v_max):

    # Calculation for joint velocites, accelerations, and jerks for each joint at each time step
    # from the calculated angles at each time step in (deg/s, deg/s^2, deg/s^3)
    
    j_velocities = derivative_array(degrees_array)
    j_accelerations = derivative_array(j_velocities)
    j_jerks = derivative_array(j_accelerations)

    # print("Velocities: ", j_velocities)
    # print("Accelerations: ", j_accelerations)
    # print("Jerks: ", j_jerks)

    # Limit Error Calculations
    # Compare values of (v, a, j) at each time step to find where 
    # and when errors will occur

    limit_errors_string = [] # Formatted list of strings for when errors occur
    limit_errors_list = [] # list of indexes for (step, joint)
    
    cnt = 0 # Counter variable for identifying type of limit

    for i in range(len(degrees_array)): # Iterate through each time step

        velocities_i = obtain_velocities(degree_array, i) # obtain the velocity list up to the current time step
        limits_i = interpolate_limits(velocities_i, limit_matrices, v_max) # Call interpolate limits to obtain the limits for this time step
        #print(limits_i) # Check to make sure the shape is correct

        for limit in limits_i:
            if cnt == 0:
                limit_result = np.greater(np.abs(j_velocities[i]), np.abs(limit))
            
            elif cnt == 1:
                limit_result = np.greater(np.abs(j_accelerations[i]), np.abs(limit))
            
            elif cnt == 2:
                limit_result = np.greater(np.abs(j_jerks[i]), np.abs(limit))


            if (np.any(limit_result)):
                
                error_indexes = np.where(limit_result)[0] # Indexes not joint numbers, add 1 for joint number
                # Formatted string to portray limit errors to user
                error_string = f"Limit type: {cnt}, Time Step: {i}, Joint(s): {error_indexes}" 
                # List format for easy acquisition of limit type, time-step, joints, and the limits exceeded
                error_list = [cnt, i, error_indexes, limits_i] # include all limits for time step to make corrections easier

                limit_errors_string.append(error_string)
                limit_errors_list.append(error_list)
            
            cnt += 1 # increment counter

        cnt = 0 # reset counter for subsequent loops

    time_step_dict = {} # Dictionary by each time step

    # Iterate through the errors list, access time step, create dictionary with
    # key = time step, value(s) = error at time step
    for error in limit_errors_list:
        time_step = error[1]  # Access the time step index
        time_step_dict.setdefault(time_step, []).append(error)

    return limit_errors_string, time_step_dict

# Function to correct any limit errors found during check limits

# Loops through the dictionary provided from check_limits and handles joint based on the 
# highest order error that occured. Calls the appropriate fix_* function to generate a new value
# for joint position


def correct_path(angles_array, time_step_dict):
     
    # Iterate through time step dictionary to obtain all errors at each time step
    for time_step in time_step_dict:
        
        type_list = [] # Initialize list to show error types at each time step, reset on new loop
        
        # Obtain error types for each time step
        for error in time_step_dict[time_step]:
            type_list.append(error[0]) # append type of error to list
            joints = error[2] # Obtain joints causing error
            limits_i = error[3] # Obtain the limits for the time step

        highest_order = max(type_list) # Obtain the highest order error present at the time step
        
        if highest_order == 0: # Velocity Error
            angles_array = fix_velo(angles_array, joints, limits_i, time_step)
        
        elif highest_order == 1: # Acceleration Error
            angles_array = fix_acc(angles_array, joints, limits_i, time_step)
        
        elif highest_order == 2: # Jerk Error
            angles_array = fix_jerk(angles_array, joints, limits_i, time_step)
        
    return angles_array # Returns new angles_array with fixed positions
    
# fix_* functions
# Called from correct_path to fix the joint positions based on highest order error

def fix_velo(angles_array, joints, limits_i, time_step):# Shouldn't ever need to be used unless the robot is trying to move too fast
    # position from velo p = v*t + p0
    dt = 0.008 # Change in time
    pct = 0.95 # Percent of limit being used in new calc

    for joint in joints:
        p0 = angles_array[time_step - 1][joint] # obtain position of joint at t-1 
        
        j_velo = limits_i[0][joint] * pct # Calculate the velocity a the time step

        p_new = (j_velo*(dt) + p0) % 360
        angles_array[time_step][joint] = p_new

    return angles_array

def fix_acc(angles_array, joints, limits_i, time_step):
    
    # normal position --> p = 1/2(a)*t^2 + v*t + p0
    dt = 0.008 # Change in time
    pct = 0.95 # percent of limit being used in new calc
    vel = derivative_array(angles_array)
    acc = derivative_array(vel)

    for joint in joints:
        
        p0 = angles_array[time_step - 1][joint] # obtain position of joint at t-1 
        j_velo = vel[time_step - 1][joint] # obtain velocity for previous motion
        j_acc = acc[time_step][joint] # obtain acceleration
        
        # Checks for sign to maintain original direction
        if j_acc > 0:
            new_acc = limits_i[1][joint] * pct # Calculate new acceleration to use based on limit
        else:
            new_acc = -1 * (limits_i[1][joint] * pct) # Calculate new acceleration to use based on limit

        print("Old Acc: ",j_acc, " New Acc: ", new_acc)
        p_new = ((1/2)*new_acc*(dt**2) + j_velo*(dt) + p0) % 360 # Calculate new angle
        angles_array[time_step][joint] = p_new # Update angle

    return angles_array

def fix_jerk(angles_array, joints, limits_i, time_step):
    
    # position including jerk --> p = 1/6(j)*t^3 + 1/2(a)*t^2 + v*t + p0
    dt = 0.008 # Change in time
    pct = 0.95 # percent of limit being used in new calc
    vel = derivative_array(angles_array)
    acc = derivative_array(vel)
    jerk = derivative_array(acc)

    for joint in joints:
        p0 = angles_array[time_step - 1][joint] # obtain position of joint at t-1 
        j_velo = vel[time_step - 1][joint] # obtain velocity for previous motion
        j_acc = acc[time_step][joint] # obtain acceleration
        j_jerk = jerk[time_step][joint] # obtain jerk
        
        # Checks for sign to retain original direction
        if j_acc > 0:
            new_acc = limits_i[1][joint] * pct # Calculate new acceleration to use based on limit
        else:
            new_acc = -1 * (limits_i[1][joint] * pct) # Calculate new acceleration to use based on limit

        print("Old Acc: ", j_acc," New Acc: ", new_acc)
        
        if j_jerk > 0:
            new_jerk = limits_i[2][joint] * pct # Calculate new jerk to use based on limit
        else:
            new_jerk = -1 * (limits_i[2][joint] * pct)
        
        print("Old Jerk: ", j_jerk," New Jerk: ", new_jerk)

        p_new = ((1/6)*new_jerk*(dt**3) + (1/2)*new_acc*(dt**2) + j_velo*(dt) + p0) % 360
        angles_array[time_step][joint] = p_new

    return angles_array
    




if __name__ == "__main__":

    # Movmement in Y-Z plane only, X calculations = Y robot and Y calculations = Z robot

    # Initialize variables used in loops and calculations
    point_list = []
    velocity_list = []
    acceleration_list = []
    q0 = robot.qr # Robot home position

    g, m, l, J, rp, Jp, M = physicalParameters() # Obtain the physical parameters
    thk, tha, oms, rs, Is = freeParameters() # Obtain the free parameters
    calK = -(Jp/(rp + rs))
    COR = 0.6

    last_th, last_dth = zeroBeam() # Sets the zero position for the beam
    print(last_th, last_dth)
    while last_th != 0:  # If theta is not 0 redo zeroing until it is
        print("redo zeroing")
        last_th, last_dth = zeroBeam()
    last_dth, omkm = initialCondition() # Set the initial condition for the beam
    if omkm is None:
        last_dth, omkm = initialCondition() # If omkm is undefined try again

    # first set of VHC parameters for the initial conditions
    Ik = Is + calK*(omkm - oms) # desired impulse value
    rk = rs # desired point of application
    RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0 = actuatorParameters(omkm, rk, Ik, COR) # we don't want to keep recomputing these
    
    j = 0 # Increment variable for obtaining path
    period = 0.79 # Period of time for each loop

    impact_ctr = 0
    tolerance = 0.003

    # While loop that obtains all cartesian positions calculated while encoder is in downward motion
    # Only obtains data up to the first collision and ignores any data points prior to downward motion
    # Centers the robots motion to the home position
    # Appends velocity to velocity_list (isn't used anymore since obtain_velocities() function was added)

    while impact_ctr < 1:

        th, dth, d2th = read_serial_data()
        print("theta: ", th)
        print("dtheta: ", dth)
        print("d2theta: ", d2th)

        X, Y, dX, dY, d2X, d2Y = desiredPosVelAccn(th, dth, d2th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
        if (j == 0):
            first_X = X
            first_Y = Y
            
            # Find distance between the first point and robot home position
            center_distance_x = 0.35 - first_X
            center_distance_y = 0.3 - first_Y

            j += 1
            # Add the centering distance to start from home position
            point = (X + center_distance_x, Y + center_distance_y)
            point_list.append(point) # Create point in order to convert to joint angles
             
            v = math.sqrt(dX**2 + dY**2) * 1000 # Keep list of velocities for limit calcluations (in mm/s)
            velocity_list.append(v)
             

        # # if dth < 0 the encoder is in downward motion
        if (j != 0):
            if (dth < 0): # Check for start of motion, ignore any values before
                if (isnum(X) and isnum(Y)): # Check for valid number, call invalid if invalid
                    if (X * 1000 - point_list[-1][0] > tolerance or Y * 1000 - point_list[-1][1] > tolerance): # Check for repeated values, ignore if repeated

                        last_X = X
                        last_Y = Y
                        last_dX = dX
                        last_dY = dY
                        last_d2X = d2X
                        last_d2Y = d2Y

                        # Add centering distance to continue from home position
                        point = (X + center_distance_x, Y + center_distance_y)
                        point_list.append(point)

                        v = math.sqrt(dX**2 + dY**2) * 1000 # Keep list of velocities for limit calcluations in (mm/s)
                        velocity_list.append(v)

                else: # don't think this ever gets used
                    X, Y, dX, dY = invalid_sig(last_X, last_Y, dX, dY, d2X, d2Y)
                    last_X = X
                    last_Y = Y
                    last_dX = dX
                    last_dY = dY

                    point = (X, Y)

                    point_list.append(point) # Append the point to the list

        if dth > 0 and dth * last_dth < 0:
            omkm = - dth
            Ik = Is + calK*(omkm - oms) # desired impulse value
            rk = rs # desired point of application
            # print("an impact has occurred")
            impact_ctr = impact_ctr + 1
            # print(impact_ctr)
            #COR = COR + 
            RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0 = actuatorParameters(omkm, rk, Ik, COR) # new VHC parameters
            # print(RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
        last_dth = dth
    
    print("Point List: ", point_list)

    
    pause() # Pause to ensure good data

    
    # Establish UDP connection
    client = UDPClient("192.168.0.3")
    client.connect()
    print("Connection Established to Robot")
    
    # Obtain Limits
    # Must obtain limits prior to initialization to obtain the correct data size
    # Loops until they are given (errors if run is cancelled before end pack)
    # Gets the limit look up table and the system max velocity
    while (True):
        
        try:
            limit_matrices, v_max = obtain_limits(client) # Obtain the no payload/max payload limit matrices
            #print(limit_matrices)
            print("Limits Length: ", len(limit_matrices))
            break

        except ValueError:

            client.send_end_pack()
            client = UDPClient("192.168.0.3")
            client.connect()

    # Inverse Kinematics
    # Utilizes joint_angles() function to obtain the SE3 matrix for each point 
    # ikine_LM() is the inverse kinematics solver takes the SE3 matrix and converts it to joint angles in radians
    # ik_sol.q is where the joint angles are contained within the return variable (np.array object)
    # Each array is then appended to a list before that is converted into another np.array for ease of 
    # future calculations

    # Loops through each point in the point list
    wpr_pos = [-164.003, 0.00, -0.001] # Initial end effector position used to calculate joint positions from ikine

    # Obtain the joint angles array from the calculated points list
    angles_list = []
    for p in point_list: # iterate over each point to find the joint angles

        T = joint_angles(p, wpr_pos)  # Convert the cartesian position into an SE3 matrix
        ik_sol = robot.ikine_LM(T)    # Inverse kinematics to find the joint angles (in rad)
        angles_list.append(ik_sol.q)  # Create new list of joint angles for each step

    angles_array = np.array(angles_list) # convert list to numpy array for easier calculations
    
    # Conversion from radians to degrees to match the units of the robot
    # Uses rad_to_degrees() function, returns a new np.array
    degree_array = rad_to_degrees(angles_array) # Convert radians to degrees for limit calcs and motion
    print(degree_array.tolist())

    # Limit Checking
    # Takes the degree_array, limit_matrices, and system variable v_max
    # Limit interpolation is called from within check_limits()
    # Returns the limit error locations in string and dictionary format
     
    # Call check_limits() to find time steps where errors occur
    limit_errors_string, time_step_dict = check_limits(degree_array, limit_matrices, v_max)
    # Call correct_path() to correct path based on limits at the time step
    degree_array = correct_path(degree_array, time_step_dict)
    
    # Check limits again to ensure proper path (Possibly need to loop this but hopefully one time through will fix)
    # remove loop for testing purposes
    # while(len(limit_errors_string) > 0):
    #     limit_errors_string, time_step_dict = check_limits(degree_array, velocity_list, limit_matrices, v_max)
    #     print("Errors Length: ", len(limit_errors_string)) # Print out the list
    #     degree_array = correct_path(degree_array, time_step_dict, velocity_list)
    
    print(len(degree_array))

    pause() # pause before motion
    
    # Stream Motion
    # Motion Initialization
    # Send initialization pack 
    resp = client.send_init_pack() 
    rob_data = resp[18:27] # indexes [18:27] are used to obtain the joint values from response packet
    print("Initialization Pack Sent")
    
    # Send initial command pack with the rob_data set to the response pack positions
    # commandpack(time_step, end_pack?, motion type, positions)
    data = commandpack([1, 0, 1, rob_data]) # Initial command pack
    resp = client.send_command_pack(data)

    # Motion Loop
    # Iterates through the created path for each time step
    for i in degree_array:
        # start_time = time.time() # Start time for the loop

        # Joints 1, 4, and 6 are not necessary for motion (joint 1 for rotation around z-axis, joint 4 for rotation of arm
        # and joint 6 for rotation of end effector)

        # Set robot data positions for each joint
        rob_data[0] = i[0] # joint 1
        rob_data[1] = i[1] # joint 2
        rob_data[2] = i[2] # joint 3
        rob_data[3] = i[3] # joint 4
        rob_data[4] = i[4] # joint 5
        rob_data[5] = i[5] # joint 6

        data  = commandpack([resp[2], 0, 1, rob_data]) # Create command pack
        resp = client.send_command_pack(data) # Send command pack and receive response packet
        
        # time_elapsed = time.time() - start_time # Elapsed time for the loop
        # sleep_time = 0.0079 - time_elapsed # Find how much time until next packet can be sent
        # time.sleep(sleep_time) # Wait until next packet can be sent

    # Final Command Pack
    data = commandpack([resp[2], 1, 1, rob_data])
    resp = client.send_command_pack(data)

    # End of Stream Pack
    client.send_end_pack()
    print("Stream Complete at command: ", resp[2])