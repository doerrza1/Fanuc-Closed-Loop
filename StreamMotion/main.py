import numpy as np
# import scipy
import serial
import time

from src.client import *
from src.utils import *
from src.display import *

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
    g = 9.81 # acceleration due to gravity (m/s^2)
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
    PSIX = 0
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
        last_th = th
        last_dth = dth
    try:
        omkm = - np.sqrt(last_dth**2 + (2*m*g*rp/Jp)*(np.sin(last_th) - np.sin(thk))) # initial pre-impact velocity
        return last_dth, omkm
    except:
        print("the initial condition is invalid -- go again")
    return last_dth, None

def initial_signal(rob_x, rob_z, init_x, init_z):
    
    half_x = abs(rob_x - init_x) /2 # Finds the value of half the difference
    half_z = abs(rob_z - init_z) /2 

    x1 = init_x + half_x
    z1 = init_z + half_z

    # Signal Definition
            
    acc_base = np.linspace(1, 125, 125)**2 # base array for acceleration with 125 signals (1 second)
    dec_base = np.linspace(125, 1, 125)**2 # base array for deceleration with 125 signals (1 second)

    # x- accel/decel
    acc_base_x = acc_base / acc_base.sum() * (rob_x - x1)
    acc_x = rob_x - np.cumsum(acc_base_x)

    dec_base_x = dec_base / dec_base.sum() * (x1 - init_x)
    dec_x = rob_x - np.cumsum(dec_base_x)

    # z- accel/decel
    acc_base_z = acc_base/ acc_base.sum() * (rob_z - z1)
    acc_z = rob_z - np.cumsum(acc_base_z)

    dec_base_z = dec_base / dec_base.sum() * (z1 - init_z)
    dec_z = rob_z - np.cumsum(dec_base_z)

    # signals
    sig_x = np.append(acc_x, dec_x)
    sig_z = np.append(acc_z, dec_z)

    signal = [sig_x, sig_z]

    return signal

    
if __name__ == "__main__":
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

    # Stream Initialization
    client = UDPClient("192.168.0.3")
    client.connect()
    print("Connection Established to Robot")

    # Send initialization pack
    resp = client.send_init_pack() 
    rob_data = resp[9:18]
    print("Initialization Pack Sent")

    while impact_ctr < 10:
        start_time = time.time()

        th, dth, d2th = read_serial_data()
        print("theta: ", th)
        print("dtheta: ", dth)
        print("d2theta: ", d2th)

        serial_time = time.time()
        print("Serial Time: ", serial_time)
        X, Y, dX, dY, d2X, d2Y = desiredPosVelAccn(th, dth, d2th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)

        calc_time = time.time()
        print("Calculation Time: ", calc_time)

        # Stream Motion
        if (j == 0): # Go to initial position

            rob_x = rob_data[0] # Robot initial x position
            rob_z = rob_data[2] # Robot initial z position
            
            init_x = X *1000    # Program initial x position 
            init_z = Y *1000    # Program initial y position

            signal = initial_signal(rob_x, rob_z, init_x, init_z) # Call initial signal

            data = commandpack(1, 0, 0, rob_data) # Initial command pack
            resp = client.send_command_pack(data)
            
            for i in range(250): # Loop to send signals
                st = time.time()

                rob_data = resp[9:18]

                rob_data[0] = signal[0][i]
                rob_data[2] = signal[1][i]

                data = commandpack([resp[2], 0, 0, rob_data])
                resp = client.send_command_pack(data)
                
                el = time.time() - st
                sleep_time = period - el
                time.sleep(sleep_time)

            j += 1 # Increment iterator to proceed to next statement group

        else: # Subsequent movemements determined by function

            rob_data = resp[9:18] # Update the robot position data to response packet values
            rob_data[0] = X *1000 # Set the x-position to desired position in mm
            rob_data[2] = Y *1000 # Set the y-position to desired position in mm

            data = commandpack([resp[2], 0, 0, rob_data]) # Create command pack [signal count, lastpack?, coordinate sys, position]
            resp = client.send_command_pack(data)
            #display_cmd_pack(resp) # Displays the command pack


        if dth > 0 and dth * last_dth < 0:
            omkm = - dth
            Ik = Is + calK*(omkm - oms) # desired impulse value
            rk = rs # desired point of application
            print("an impact has occurred")
            impact_ctr = impact_ctr + 1
            print(impact_ctr)
            #COR = COR + 
            RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0 = actuatorParameters(omkm, rk, Ik, COR) # new VHC parameters
            print(RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
        last_dth = dth

        loop_time = time.time() - start_time
        print("Loop Time: ", loop_time)

        sleep_time = period - loop_time
        time.sleep(sleep_time) # Pauses loop to ensure not too many packs are sent
        print('____________________________')
