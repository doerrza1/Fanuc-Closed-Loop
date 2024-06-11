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

def display_limit_pack(data):
    print("Packet Type: ", data[0])
    print("Version Number: ", data[1])
    print("Axis Number: ", data[2])
    print("Type of Limit: ", data[3])  #0 = vel, 1 = acc, 2 = jerk
    print("Vmax (mm/s): ", data[4])
    print("Check Time (sec): ", data[5])
    print("Allowable limit: ", data[6])