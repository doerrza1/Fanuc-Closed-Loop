import struct

def initpack():
    data=[0,1]
    pack=''.encode()
    for i in data:
        pack+=struct.pack('>I',i)
    return pack

def endpack():
    data=[2,1]
    pack=''.encode()
    for i in data:
        pack+=struct.pack('>I',i)
    return pack

def explainRobData(data):
    # Simplified pack/unpack to reduce time/function calls
    pack = list(struct.unpack('>IIIBBHHHI' + 'f'*27, data))
    return pack

def commandpack(data):

    # Simplified pack to reduce function calls
    sequence_no = data[0]
    last_data = data[1]
    data_style = data[2]
    jnt_data = data[3]
    
    pack = struct.pack('>IIIBBHHBBHHHH' + 'f'*9,
                       1, 1, sequence_no, last_data, 0, 0, 0, data_style, 0, 0, 0, 0, 0,
                       *jnt_data)
    return pack


def getStatus(data):
    accept_cmd = (data[3] & 0b0001)>0
    received_cmd = (data[3] & 0b0010)>0
    sysrdy = (data[3] & 0b0100)>0
    rbt_inmotion = (data[3] & 0b1000)>0
    status = [accept_cmd, received_cmd, sysrdy, rbt_inmotion]
    return status

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
    
    # Unpack for the structure of limit response packet
    pack=list(struct.unpack('>IIIIII'+'f'*40, data))
    return pack