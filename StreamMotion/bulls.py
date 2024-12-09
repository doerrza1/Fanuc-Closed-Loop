# 3rd version of final deliverable

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

robot = rtb.models.URDF.id7l() # Load in the robot from URDF file class
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

# If the value obtained is invalid calculate the next value to prevent a jerk error
def invalid_sig(X, Y, dX, dY, d2X, d2Y, dt = 0.008, g = 9.81):
    
    # new velocity = velocity + acceleration * change in time 
    new_dX = dX + d2X * dt
    new_dY = dY + d2Y * dt

    # new position = position + new velocity * change in time
    X = X + new_dX * dt
    Y = Y + new_dY * dt

    return X, Y, new_dX, new_dY

def isnum(num):
    try:
        float(num)
        return True
    
    except:
        return False
    
def initial_signal(rob_x, rob_y, init_x, init_y, num):
    
    half_x = abs(rob_x - init_x) /2 # Finds the value of half the difference
    half_y = abs(rob_y - init_y) /2 

    x1 = init_x + half_x
    y1 = init_y + half_y

    # Signal Definition
            
    acc_base = np.linspace(1, num, num)**2 # base array for acceleration with 125 signals (2 seconds)
    dec_base = np.linspace(num, 1, num)**2 # base array for deceleration with 125 signals (2 seconds)

    # x- accel/decel
    acc_base_x = acc_base / acc_base.sum() * (rob_x - x1)
    acc_x = rob_x - np.cumsum(acc_base_x)

    dec_base_x = dec_base / dec_base.sum() * (x1 - init_x)
    dec_x = x1 - np.cumsum(dec_base_x)

    # y- accel/decel
    acc_base_y = acc_base/ acc_base.sum() * (rob_y - y1)
    acc_y = rob_y - np.cumsum(acc_base_y)

    dec_base_y = dec_base / dec_base.sum() * (y1 - init_y)
    dec_y = y1 - np.cumsum(dec_base_y)

    # signals
    sig_x = np.append(acc_x, dec_x)
    sig_y = np.append(acc_y, dec_y)

    # signal = [sig_x, sig_z]

    return sig_x, sig_y

def end_effector(start):
    end = 90
    diff = (start - end) / 2
    half = end + diff

    acc_base = np.linspace(1, 250, 250)**2 # base array for acceleration with 125 signals (1 second)
    dec_base = np.linspace(250, 1, 250)**2 # base array for deceleration with 125 signals (1 second)

    #accel/decel
    acc_base_th = acc_base / acc_base.sum() * (start - half)
    acc_th = start - np.cumsum(acc_base_th)
    
    dec_base_th = dec_base / dec_base.sum() * (half - end)
    dec_base_th = half - np.cumsum(dec_base_th)

    end_sig = np.append(acc_th, dec_base_th)
    return end_sig


# Create an array to add onto end of each pass to stop motion
def decelerate(x_velo, y_velo):

    x_sig = np.linspace(start = x_velo, stop = 0, num = 75)
    y_sig = np.linspace(start = y_velo, stop = 0, num = 75)

    return x_sig, y_sig

def interpolate(point_list, max_velo):
    fixed_points = [point_list[0]] # Create new list starting with first point

    for i in range(len(point_list) - 1):

        p1 = point_list[i]
        p2 = point_list[i + 1]

        velo = check_velo(p1, p2)

        if (velo > max_velo):

            num_intermediate = int(velo // max_velo)
            dx = (p2[0] - p1[0]) / (num_intermediate + 1)
            dy = (p2[1] - p1[1]) / (num_intermediate + 1)

            for j in range(1, num_intermediate + 1):
                new_x = p1[0] + j * dx
                new_y = p1[1] + j * dy

                if (new_x, new_y) != fixed_points[-1]:

                    fixed_points.append((new_x, new_y))
        
        if p2 != fixed_points[-1]:

            fixed_points.append(p2)


    return fixed_points

def check_velo(p1, p2):

    velo = math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p2[0]) ** 2)
    return velo

if __name__ == "__main__":
    X_list = []
    Y_list = []
    point_list = []

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
             point = (X * 1000, Y * 1000)
             point_list.append(point)

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

                        point = (X * 1000, Y * 1000)
                        point_list.append(point)

                else:
                    X, Y, dX, dY = invalid_sig(last_X, last_Y, dX, dY, d2X, d2Y)
                    last_X = X
                    last_Y = Y
                    last_dX = dX
                    last_dY = dY

                    point = (X * 1000, Y * 1000)

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
    print(len(point_list))

    # max_velo = 1.8 # 1.8 mm/8ms = 225 mm/s
    # encoder_sig = interpolate(point_list, max_velo) # Call interpolate to ensure smooth signal

    # print("Encoder Signal: ", encoder_sig)
    # print("Length Encoder Signal", len(encoder_sig))
    pause() # Pause program before stream motion begins

    # Steam Motion
    # Stream Initialization
    client = UDPClient("192.168.0.3")
    client.connect()
    print("Connection Established to Robot")

    # Send initialization pack
    resp = client.send_init_pack() 
    rob_data = resp[9:18]
    print("Initialization Pack Sent")

    init_x = point_list[0][0]
    init_y = point_list[0][1]

    rob_x = rob_data[1]
    rob_y = rob_data[2]

    print("Initial X ", init_x)
    print("Initial Z ", init_y)
    print("Robot X ", rob_data[1])
    print("Robot Y ", rob_data[2])

    sig_x, sig_y = initial_signal(rob_data[1], rob_data[2], init_x, init_y, 250) # Obtain initial signal
    eoat_sig = end_effector(rob_data[3])
    encoder_x, encoder_y = initial_signal(point_list[0][0], point_list[0][1], point_list[-1][0], point_list[-1][1], 125)


    data = commandpack([1, 0, 0, rob_data]) # Initial command pack
    resp = client.send_command_pack(data)
    
    print("Initial Movement Begin")
    for i, j in zip(sig_x, sig_y):
        rob_data = resp[9:18]

        rob_data[1] = i # Send x_values to robot (actually z-axis)
        # rob_data[2] = j # Send y_values to robot (actually y-axis)

        data = commandpack([resp[2], 0, 0, rob_data]) # Create data pack
        resp = client.send_command_pack(data) # send data pack
    print("Initial Movement Complete")

    # print("EOAT Movement Begin")
    # for i in (eoat_sig):

    #     rob_data = resp[9:18]

    #     rob_data[3] = i

    #     data = commandpack([resp[2], 0, 0, rob_data]) # Create data pack
    #     resp = client.send_command_pack(data) # send data pack

    # print("EOAT Movement complete")
    
    print("Encoder to Stream Begin")
    for i in point_list:
        rob_data = resp[9:18] # Obtaint the cartesian data from response packet
        
        # Modify cartesian data to the points determined by the equation
        rob_data[1] = i[0]
        # rob_data[2] = i[1]

        data = commandpack([resp[2], 0, 0, rob_data])
        resp = client.send_command_pack(data)

    print("Encoder to Stream End")

    # Using first and last points to determine motion
    # print("Encoder to Stream Begin")
    # for i, j in zip(encoder_x, encoder_y):
    #     rob_data = resp[9:18] # Obtaint the cartesian data from response packet
        
    #     # Modify cartesian data to the points determined by the equation
    #     rob_data[1] = i
    #     rob_data[2] = j

    #     data = commandpack([resp[2], 0, 0, rob_data])
    #     resp = client.send_command_pack(data)

    # print("Encoder to Stream End")

    x_decel, y_decel = decelerate(dX, dY)
    print("Deceleration begin")
    for i, j in zip(x_decel, y_decel):
        rob_data = resp[9:18]

        rob_data[1] += i
        # rob_data[2] += j

        data = commandpack([resp[2], 0, 0, rob_data])
        resp = client.send_command_pack(data)

    print("Deceleration End")

    data = commandpack([resp[2], 1, 0, rob_data])
    resp = client.send_command_pack(data)

    client.send_end_pack()


    # plot(x_tot, len(x_tot))
    # plot(z_tot, len(z_tot))



    
       