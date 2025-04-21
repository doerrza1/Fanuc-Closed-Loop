Stream Motion Manual

Teach Pendant

Use the Stream Teach Pendant script to start stream motion. Robot must be near home position (X: 0, Y: 350, Z: 300, W: -164, P: 0, R: 0) in order for stream to start without faulting. After program is started it will wait for connection and initialization from the github packages to begin motion. 

Support Packages from github

client.py

client.py defines the UDPClient class and functions used to communicate with the robot. Requires socket and numpy add ons and import utils.

__init__(self, ip, port = 60015)

Initializes the UDPClient object when given an ip address, port number is set constant at 60015 for stream motion
Creates socket object in order to establish communication link to robot

connect(self)
Connects to the robot through the socket (needs to be included before any commands can start)

send_init_pack(self)
Calls initpack (see utils.py)
Sends initialization pack to the robot to initialize communication link
resp is the response from the robot through the recv(receive) command (132 is the buffer for the 132 bit communication)
Returns the response using the explainRobotData function (see utils.py)

send_command_pack(self, data)
Given a data pack will send it to the robot
Receives the response and uses explainRobotData function 

send_end_pack(self)
Called at the end of communication to signal to robot the connection can close
Calls end_pack (see utils.py) and sends to robot
Closes the socket

utils.py

utils.py contains the functions used to create packs and unpack data returned by the robot. Requires struct in order to convert to and from binary.

initpack()
Creates and returns the initialization pack 

endpack()
Creates and returns the end pack

explainRobData(data)

Given the return communication from the robot (in binary) this function parses out each value obtained from the data and converts to int/float/char and appends them to a list.
List format = [ packet type, version #, sequence #, status, read io type, read io index, read io mask, read io value, time stamp, X (mm), Y (mm), Z (mm), W (deg), P (deg), R (deg), ext axis 1, ext axis 2, ext axis 3, J1, J2, J3, J4, J5, J6, J7, J8, J9, J1 motor current, J2 motor current, J3 motor current, J4 motor current, J5 motor current, J6 motor current, J7 motor current, J8 motor current, J9 motor current ] 
Cartesian points at indexes 9:15
Joint positions at indexes 18:27
Returns the list

commandpack(data)

data is a list given made up of [sequence data, last command data, cartesian movement = 0 or joint movement = 1, [cartesian or joint positions]]
Creates the command pack placing the given values in their proper place and converting to binary.
Returns the pack generated

Support Packages by Zach

fullvelo.py

Support package to obtain velocity, acceleration/deceleration time, and steady state time for all axes/joints.

motion(m)

Function to create a list of lists comprised of lists [ velocity, acceleration/deceleration time, steady state motion time ] when given the type of motion (m)
m = 0 for cartesian, m = 1 for joint
Asks user which joints/axes to move, if y is inputted will call velo(m, joint/axis) to create the list
If not chosen will fill in 0’s for all values
returns list of list for all joints
	
velo(m, axis/joint) - called by motion() no need to call it yourself

Function to create the lists for motion(m) given the type of motion and the axis/joint
Prompts the user for a velocity value within a specified range for cartesian or loops until valid velocity is inputted for joint motion
Prompts the user for acceleration/deceleration time and steady state time in seconds and converts that into the number of signals required (1 second = 125 signals)
returns list of [ velocity, acceleration/deceleration time, steady state motion time ]

signal.py

Support Package to create signals for each joint/axis given the list of lists provided by motion(m)

create(lol)

Function to create arrays of values to increment by (effectively velocity values) for each joint/axis
Uses np.linspace(start = 0, stop = velocity, num = accel/deccel time) to create the acceleration array with evenly spaced values between 0 and velocity
Uses np.flip() to create deceleration array equal and opposite to acceleration
Uses np.full(steady state time, velocity) to create array of steady state time length filled with the velocity value
Appends the 3 signals together to create the signal and creates a reverse array using np.flip()/multiplying by -1 to return robot to original position
Appends each signal to a new list
Calls normalize() 
returns the normalized list of lists

normalize(lol)

Function to ensure each signal array is the same length to prevent out of bounds errors during transmission
Finds the length of the longest array present within the list of lists
Loops through the list of lists and finds the difference between the length of each array and the maximum length
If the length is less than the maximum length uses np.pad() to add 0’s to either the end or beginning of array depending on the user input
Appends padded arrays to a new list and returns that to signal

display.py

Support package to display response packets from the robot given the response data

display_cmd_pack()
Displays data for cartesian movement

display_limit_pack()
Displays data for either velocity, acceleration, or jerk limit packs

display_jnt_pack()
Displays data for joint movement


limit.py

Support package to create command packs in order to obtain limit data from the robot given an axis or joint, use send_command_pack() to send

velocitypack()
Creates command pack to obtain velocity limit data

accelerationpack()
Creates command pack to obtain acceleration limit data

jerkpack()
Creates command pack to obtain jerk limit data

explainLimitResponse()
Given a response pack from any pack above will create a list of all values returned in readable format (i.e. not binary)
Use display_limit_pack() in order to print the values in a nice way


radius.py

Support package to give an approximate radius of motion for each joint using the initial position, used in conversions from deg/s to mm/s to ensure the deg/s value falls within the 255 mm/s limit.

radius(joint)
returns the radius for a specified joint 

Stream Motion Main Files

All files that start with c are for cartesian movement, start with j are for joint movement.

-stream.py
Basic files used to move a specified joint/axis from a np.linspace() signal
Not very useful for anything just used to understand how each type of motion works


-velocity.py
Files used to move a specified joint/axis at a steady state velocity after accelerating
Uses user inputs for values

-acceleration.py
Files used to move a specified joint/axis at a steady state velocity after a specified acceleration time
Uses user inputs for values

fullmotion.py
File used to control every joint/axis with a user inputted velocity, acceleration/deceleration, and steady state time. 
Uses use inputs for values
Uses aspects from all files to create a “Master Control” over every joint

Instructions for main

Make sure all files are imported as well as numpy
Define a function used that will determine the path of the robot
Create the UDPClient object and connect it to the robot
Send the initialization pack using send_init_pack(), receive the robot current position from the return value of send_init_pack()
Optional: print out the data wanted for cartesian position [8:15] or joint position [18:27]
Enter into for loop to constantly send command packs and receive robot data. End each loop with send_command_pack(data) with data being binary and by parsing out either the cartesian or joint data for the subsequent loop. Each if statement should create the data pack using commandpack(data) with data in list format.
First loop sends initial command pack [1, 0, cartesian=0 or joint=1, [positions]] (if statement with i == 0)
Subsequent loops send command pack of [resp[2], 0, cartesian=0 or joint=1, [positions]] (else if statement i < # of signals - 1)
Final command sends command pack of [resp[2], 1, cartesian=0 or joint=1, [positions]]
Send the end pack using send_end_pack()
Congrats you should theoretically have a working code


Limits

Utilizes the limit.py file for obtaining limits lookup table and interpolation of limits based on system max velocity and peak velocity within the last 5 seconds

obtain_limits(client)
Given the connection to the robot this function sends velocity, acceleration, and jerk limit request packs to the robot
After all responses have been obtained they are organized into a matrix for each type of limit
Returns the a matrix of limit look up table
limit_matrices = [no_load_vel_matrix, max_load_vel_matrix, no_load_acc_matrix, max_load_acc_matrix, no_load_jerk_matrix, max_load_jerk_matrix]

interpolate_limits(velocities, limit_matrices, v_max)
Given the list of end effector velocities (mm/s), the limits matrices, and the system max velocity it will interpolate the limits for every joint at the peak velocity within the last 5 seconds
After interpolating the limits it will return a list of lists containing of size 6 with limits for each joint
List = [velocity_limits, acceleration_limits, jerk_limits]

interpolate_limits() is called from within the check_limits() function inside of the main python file.

Roboticstoolbox

robot = rtb.models.URDF.id7l()
Initialize the robot object loaded in from the urdf file within rtb.data (this is already within the file path on the stream computer) 
Robot object is used for inverse kinematics calculations

T = sm.SE3(X, Y, Z) * sm.SE3.RPY([W,P,R], order='zyx')
Creates SE3 matrix given robot XYZ and WPR position 
Used for inverse kinematics solver

ik_sol = robot.ikine_LM(T)
Given SE3 matrix will solve for the inverse kinematics
From ik_sol, ik_sol.q will give you joint angles

J = robot.jacob0(q)
ee_velo = J @ qd
With robot current joint position (q) in radians and current joint velocities (qd) creates the jacobian matrix and then finds the end effector velocities (m/s) in XYZ coordinates
linear_velo = np.sqrt(ee_velo[0]**2 + ee_velo[1]**2 + ee_velo[2]**2) * 1000 to convert into a magnitude value of velocity in mm/s
Used in obtain velocities function to get current end effector velocities
