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

arduino_port = '/dev/ttyACM0'
baud_rate = 115200
timeout = 0.005
ser = serial.Serial(arduino_port, baud_rate, timeout=timeout)

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

def create_signal(first_point, last_point, num):

    # Find half of the difference between points 
    half_y = abs(first_point[1] - last_point[1]) /2 
    half_x = abs(first_point[0] - last_point[0]) /2
    
    # Check for greater value, find halfway point
    if (first_point[0] > last_point[0]):
        x1 = last_point[0] + half_x
    else:
        x1 = first_point[0] + half_x

    if(first_point[1] > last_point[1]):
        y1 = last_point[1] + half_y
    else:
        y1 = first_point[1] + half_y

    # Signal Definition
            
    acc_base = np.linspace(1, num, num)**2 # base array for acceleration with 125 signals (2 seconds)
    dec_base = np.linspace(num, 1, num)**2 # base array for deceleration with 125 signals (2 seconds)

    # x- accel/decel
    acc_base_x = acc_base / acc_base.sum() * (first_point[0] - x1)
    acc_x = first_point - np.cumsum(acc_base_x)

    dec_base_x = dec_base / dec_base.sum() * (x1 - last_point[0])
    dec_x = x1 - np.cumsum(dec_base_x)

    # y- accel/decel
    acc_base_y = acc_base/ acc_base.sum() * (first_point[1] - y1)
    acc_y = first_point[1] - np.cumsum(acc_base_y)

    dec_base_y = dec_base / dec_base.sum() * (y1 - last_point[1])
    dec_y = y1 - np.cumsum(dec_base_y)

    # signals
    sig_x = np.append(acc_x, dec_x)
    sig_y = np.append(acc_y, dec_y)

    return sig_x, sig_y


if __name__ == "__main__":

    # Movmement in Y-Z plane only, X calculations = Y robot and Y calculations = Z robot
    point_list = []
    velocity_list = []
    acceleration_list = []
    
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
    
    j = 0 # Increment variable for initial movement
    period = 0.79 # Period of time for each loop

    impact_ctr = 0
    tolerance = 0.003

    while impact_ctr < 1:

        th, dth, d2th = read_serial_data()
        print("theta: ", th)
        print("dtheta: ", dth)
        print("d2theta: ", d2th)

        X, Y, dX, dY, d2X, d2Y = desiredPosVelAccn(th, dth, d2th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
        if (j == 0):
             first_X = X
             first_Y = Y
             j += 1
             point = (X, Y)
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

                        point = (X, Y)
                        point_list.append(point)

                        v = math.sqrt(dX**2 + dY**2) * 1000 # Keep list of velocities for limit calcluations in (mm/s)
                        velocity_list.append(v)

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
    num = int(input("Enter the length for acceleration/deceleration (sec): ")) * 125
    
    initial_point = [350, 300]
    final_point = [point_list[-1][0]*1000, point_list[-1][1]*1000] # Obtain final point (in mm)

    # Create signal for motion from robot home pos to final calculated point
    signal_x, signal_y = create_signal(initial_point, final_point, num)  
    
    pause() # Pause to ensure good data


    # Stream Motion
    # Establish UDP connection
    client = UDPClient("192.168.0.3")
    client.connect()
    print("Connection Established to Robot")

    resp = client.send_init_pack() 
    rob_data = resp[18:27]
    print("Initialization Pack Sent")
    
    data = commandpack([1, 0, 1, rob_data]) # Initial command pack
    resp = client.send_command_pack(data)

    # Loop for motion
    for i, j in zip(signal_x, signal_y):
        
        rob_data = resp[9:18]

        rob_data[1] += i
        rob_data[2] += j

        data = commandpack([resp[2], 0, 0, rob_data])
        resp = client.send_command_pack(data)

    # Final Command Pack
    data = commandpack([resp[2], 1, 1, rob_data])
    resp = client.send_command_pack(data)

    # End of Stream Pack
    client.send_end_pack()
    print("Stream Complete at command: ", resp[2])

    