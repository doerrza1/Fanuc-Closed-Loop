
import math
import numpy as np
from src.client import *

# Finds the limit lookup table by looping through each joint and each type of limit
# send_*_pack functions are located in client.py
# This function MUST be called prior to stream initialization or robot will only send partial amounts of the 
# limits tables
# No fancy stuff is needed when this is called it handles it by itself

def obtain_limits(client):
    # Create a numpy matrix for each type of limit (vel, acc, jerk) for both the no payload and max payload cases
    # The numpy matrixes allow for easier interpolation of limits all at the same time without looping
    # returns a list of all matrices in order of type [vel (both cases), acc (both cases), jerk (both cases)]

    # List initialization
    no_payload_vel = []
    max_payload_vel = []
    no_payload_acc = []
    max_payload_acc = []
    no_payload_jerk = []
    max_payload_jerk = []

    for i in range(1,7): # Iterate through each joint
        
        for j in range(3): # 3 iterations per joint
            
            if (j == 0): # velocity limit
                resp = client.send_vel_pack(i)
                no_payload = resp[6:26] # Contains the first 20 floats
                max_payload = resp[26:] # Contains the last 20 floats # obtains the values after

                # Convert the lists into numpy arrays
                no_payload_array = np.array(no_payload)
                max_payload_array = np.array(max_payload)

                no_payload_vel.append(no_payload_array)
                max_payload_vel.append(max_payload_array)


            elif (j == 1): # acceleration limit
                resp = client.send_acc_pack(i)
                no_payload = resp[6:26] # Contains the first 20 floats
                max_payload = resp[26:] # Contains the last 20 floats

                # Convert the lists into numpy arrays
                no_payload_array = np.array(no_payload)
                max_payload_array = np.array(max_payload)
                
                no_payload_acc.append(no_payload_array)
                max_payload_acc.append(max_payload_array)


            else: # jerk limit
                resp = client.send_jerk_pack(i)
                no_payload = resp[6:26] # Contains the first 20 floats
                max_payload = resp[26:] # Contains the last 20 floats
                
                # Convert the lists into numpy arrays
                no_payload_array = np.array(no_payload)
                max_payload_array = np.array(max_payload)

                no_payload_jerk.append(no_payload_array)
                max_payload_jerk.append(max_payload_array)
        
        print(f"Joint {i} limits obtained")
            
    # Convert all lists of arrays into matrices
    no_load_vel_matrix = np.array(no_payload_vel)
    max_load_vel_matrix = np.array(max_payload_vel)
    no_load_acc_matrix = np.array(no_payload_acc)
    max_load_acc_matrix = np.array(max_payload_acc)
    no_load_jerk_matrix = np.array(no_payload_jerk)
    max_load_jerk_matrix = np.array(max_payload_jerk)

    # Create the list of matrices 
    limit_matrices = [no_load_vel_matrix, max_load_vel_matrix, no_load_acc_matrix, max_load_acc_matrix,
        no_load_jerk_matrix, max_load_jerk_matrix]

    v_max = resp[2] # Obtain the system max velocity provided through limit packs

    return limit_matrices, v_max 

# The big one
# This function utilizes the given system max velocity from the limit packs and a list of all 
# necessary velocities (obtained from obtain_velocities function) to find the limit based on the percentage

def interpolate_limits(velocities, limit_matrices, v_max, payload_mass = 1.5):
    
    # Finds the Vpeak from the last 5 seconds and interpolates the limits using the payload mass percent and velocity percent
    
    # System Variables
    v_min = v_max / 20 # System variable based on the Maximum Velocity

    v_peak = max(abs(v) for v in velocities) # Obtain the peak velocity from the last 5 seconds
    
    # Calculate the numerator and denominator for index calculation
    num = v_peak * 1.2 - v_min
    denom = 1/(v_max - v_min)

    # Finds the index of the velocity (0, 19) based on the percentage compared to max and min velocities
    pct = min(max(num * denom, 0.0) * 19.0, 19.0)
    idx = pct # index of the limit (ex. 18.5)
    idx_l = max(int(pct), 0) # lower index (ex. 18)
    idx_u = min(int(math.ceil(pct)), 19) # upper index (ex. 19)
    idx_frac = pct - float(int(pct)) # Obtains the difference between the pct and the index

    # Indexing the Limits lookup table at the calculated indexes
    
    # By using [:, idx_*] limit for each joint is found at once by grabbing the proper columns
    # from the matrix (this is a numpy functionality for arrays)
    
    # Difference between upper and lower value is multiplied by the index_frac to interpolate to the proper point 
    # in between those indexes

    # Calculate the no-load limits
    vel_limit_no_load = idx_frac * (limit_matrices[0][:, idx_u] - limit_matrices[0][:, idx_l]) + limit_matrices[0][:, idx_l]
    acc_limit_no_load = idx_frac * (limit_matrices[2][:, idx_u] - limit_matrices[2][:, idx_l]) + limit_matrices[2][:, idx_l]
    jerk_limit_no_load = idx_frac * (limit_matrices[4][:, idx_u] - limit_matrices[4][:, idx_l]) + limit_matrices[4][:, idx_l]

    # Calculate the full-load limits
    vel_limit_full_load = idx_frac * (limit_matrices[1][:, idx_u] - limit_matrices[1][:, idx_l]) + limit_matrices[1][:, idx_l]
    acc_limit_full_load = idx_frac * (limit_matrices[3][:, idx_u] - limit_matrices[3][:, idx_l]) + limit_matrices[3][:, idx_l]
    jerk_limit_full_load = idx_frac * (limit_matrices[5][:, idx_u] - limit_matrices[5][:, idx_l]) + limit_matrices[5][:, idx_l]

    # Calculate payload percentage and apply limits based on payload

    # Takes the limit arrays obtained and multiplies them by the percentage of payload/max payload
    # *_limit_full_load < *_limit_no_load so this lowers the limit as payload % is increased

    full_payload = 7 # maximum payload
    payload_pct = payload_mass/full_payload # percentage of payload compared to the maximum

    vel_limit = payload_pct * (vel_limit_full_load - vel_limit_no_load) + vel_limit_no_load
    acc_limit = payload_pct * (acc_limit_full_load - acc_limit_no_load) + acc_limit_no_load
    jerk_limit = payload_pct * (jerk_limit_full_load - jerk_limit_no_load) + jerk_limit_no_load

    # Create a list of arrays
    # Each array will be of size 6 containing the limits for each joint at the given v_peak

    limits = [vel_limit, acc_limit, jerk_limit]

    return limits # return the list



    

