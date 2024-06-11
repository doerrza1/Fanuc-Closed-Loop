import struct

# create request packet for max allowable velocity 
def velocitypack(axis):
    data = [3, 1, axis, 0]
    pack= ''.encode()
    for i in data:
        pack += struct.pack('>I', i)
    return pack

#create request packet for max allowable acceleration
def accelerationpack(axis):
    data = [3, 1, axis, 1]
    pack= ''.encode()
    for i in data:
        pack += struct.pack('>I', i)
    return pack

#create request packet for max allowable jerk
def jerkpack(axis):
    data = [3, 1, axis, 2]
    pack= ''.encode()
    for i in data:
        pack += struct.pack('>I', i)
    return pack

# explains the return pack given from the robot (for any of the above requests)
def explainLimitResponse(data):
    pack=list([])
    #packet type
    temp = struct.unpack('>I', data[0:4])
    pack.append(temp[0])
    #version number
    temp = struct.unpack('>I', data[4:8])
    pack.append(temp[0])
    #axis number
    temp = struct.unpack('>I', data[8:12])
    pack.append(temp[0])
    #type of limit
    temp = struct.unpack('>I', data[12:16])
    pack.append(temp[0])
    #Vmax
    temp = struct.unpack('>I', data[16:20])
    pack.append(temp[0])
    #intermediate check time
    temp = struct.unpack('>I', data[20:24])
    pack.append(temp[0])
    #limit at no payload(1)
    temp = struct.unpack('>f', data[24:28])
    pack.append(temp[0])
    #limit at no payload(2)
    temp = struct.unpack('>f', data[28:32])
    pack.append(temp[0])
    #limit at no payload(19)
    temp = struct.unpack('>f', data[32:36])
    pack.append(temp[0])
    #limit at no payload(2)
    temp = struct.unpack('>f', data[36:40])
    pack.append(temp[0])
    #limit at max payload(1)
    temp = struct.unpack('>f', data[40:44])
    pack.append(temp[0])
    #limit at max payload(2)
    temp = struct.unpack('>f', data[44:48])
    pack.append(temp[0])
    #limit at max payload(19)
    temp = struct.unpack('>f', data[48:52])
    pack.append(temp[0])
    #limit at max payload(20)
    temp = struct.unpack('>f', data[52:56])
    pack.append(temp[0])

    return pack
