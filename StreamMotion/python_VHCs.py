import numpy as np
import scipy
import serial
import time
from src.client import *
from src.utils import *
from src.display import *

arduino_port = '/dev/ttyACM0'
baud_rate = 9600
timeout = 1
ser = serial.Serial(arduino_port, baud_rate, timeout=timeout)

def read_serial_data():
    try:
        # Read a line from the serial port
        line = ser.readline().decode('utf-8').strip()
        if line:
            # Split the line into position, velocity, and acceleration
            position, velocity, acceleration = line.split(',')
            return float(position), float(velocity), float(acceleration)
    except Exception as e:
        print(f"Error reading serial data: {e}")
    return 0, 0, 0

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
    
# Initialize serial connection
ser = serial.Serial(arduino_port, baud_rate, timeout=timeout)
time.sleep(1)
ser = serial.Serial('/dev/ttyACM0', 9600)  # Replace 'COMX' with your Arduino's serial port

# Obtain Parameters 
g, m, l, J, rp, Jp, M = physicalParameters()
thk, tha, oms, rs, Is = freeParameters()
calK = -(Jp/(rp + rs))

# Initialize connection to robot
client = UDPClient("192.168.0.3")
client.connect()
print("Connection Established to Robot")

# Send initialization pack
resp = client.send_init_pack() 
rob_data = resp[9:18]
print("Initialization Pack Sent")

# initial conditions
while True:
    
    th, dth, d2th = read_serial_data()
    print(th, dth, d2th)
    time.sleep(2)
    
    omkm = - np.sqrt((2*m*g*rp/Jp)*(np.sin(th) - np.sin(thk))) # initial pre-impact velocity
    omkm = -4
    print(omkm)
    
    COR = 0.6 # initial estimate of COR
    Ik = Is + calK*(omkm - oms) # desired impulse value
    rk = rs # desired point of application
    RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0 = actuatorParameters(omkm, rk, Ik, COR) # we don't want to keep recomputing these in RT
    print(RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
    
    last_dth = dth # the last saved value of dth
    
    t0 = time.time()
    t  = t0
    impact_ctr = 0
    
    while impact_ctr < 10:
        
        th, dth, d2th = read_serial_data()
        X, Y, dX, dY, d2X, d2Y = desiredPosVelAccn(th, dth, d2th, omkm, Ik, rk, COR, RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
        print(X, Y, dX, dY, d2X, d2Y)
        
        # Stream Motion
        rob_data = resp[9:18] # Update the robot position data to response packet values
        rob_data[0] = X  # Set the x-position to desired position
        rob_data[1] = Y  # Set the y-position to desired position

        data = commandpack([resp[2], 0, 0, rob_data]) # Create command pack [signal count, lastpack?, coordinate sys, position]
        resp = client.send_command_pack(data)
        display_cmd_pack(resp) # Displays the command pack

    data = commandpack([resp[2], 1, 0, rob_data]) # Final command pack
    resp = client.send_command_pack(data) # Send final command pack
    print("Final Command Pack Sent")

    client.send_end_pack() # Sends end pack to terminate command normally
    print("End of Stream")


    '''if dth > 0 and dth * last_dth < 0:
        omkm = - dth
        Ik = Is + calK*(omkm - oms) # desired impulse value
        rk = rs # desired point of application
        print("an impact has occurred")
        impact_ctr = impact_ctr + 1
        #COR = COR + 
        RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0 = actuatorParameters(omkm, rk, Ik, COR) # new VHC parameters
        print(RX, OMX, PSIX, RX0, RY, OMY, PSIY, RY0)
    last_dth = dth'''
