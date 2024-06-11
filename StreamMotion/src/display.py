# Display function for packs

# Command Pack response

def display_cmd_pack(data):
    print("Packet Type: ", data[0])
    print("Version Number: ", data[1])
    print("Sequence Number: ", data[2])
    print("X (mm): ", data[9])
    print("Y (mm): ", data[10])
    print("Z (mm): ", data[11])
    print("W (deg): ", data[12])
    print("P (deg): ", data[13])
    print("R (deg): ", data[14])
    print("----------------------------")

def display_limit_pack(data):
    print("Packet Type: ", data[0])
    print("Version Number: ", data[1])
    print("Axis Number: ", data[2])
    
    if (data[3] == 0):
        limit = "Velocity (mm/s)"
    elif (data[3] == 1):
        limit = "Acceleration (mm/s^2)"
    else:
        limit = "Jerk (mm/s^3)"

    print("Type of Limit: ", limit)
    print("Vmax (mm/s): ", data[4])
    print("Check Time (sec): ", data[5])
    print("Allowable limit: ", data[6])
    print("----------------------------")

def display_jnt_pack(data):
    print("Packet Type: ", data[0])
    print("Version Number: ", data[1])
    print("Sequence Number: ", data[2])
    print("Joint 1 (deg): ", data[18])
    print("Joint 2 (deg): ", data[19])
    print("Joint 3 (deg): ", data[20])
    print("Joint 4 (deg): ", data[21])
    print("Joint 5 (deg): ", data[22])
    print("Joint 6 (deg): ", data[23])
    print("Joint 7 (deg): ", data[24])
    print("Joint 8 (deg): ", data[25])
    print("Joint 9 (deg): ", data[26])
    print("----------------------------")
