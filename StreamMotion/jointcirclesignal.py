import socket
import time
from src.utils import *
from src.client import *
import numpy as np
from src.plot import *
import matplotlib


# signal definition for j1  (0 -> 45 -> 0 -> -45 -> 0)

    #using linspace creates an array of numbers evenly spaced between start and stop
    #causes the robot to accelerate (start = 0, stop = 45) or decelerate(start = 45, stop = 0)
    #by incrementing by a higher (acc) or lower (dec)

#positive increment from center to end
jnt_motion1_acc = np.linspace(start = 0, stop = 1, num = 1000)
jnt_motion1_acc = jnt_motion1_acc * (22.5/sum(jnt_motion1_acc))

jnt_motion1_dec = np.linspace(start = 1, stop = 0, num = 1000)
jnt_motion1_dec = jnt_motion1_dec * (22.5/sum(jnt_motion1_dec))

#negative increment from end to end
jnt_motion2_acc = np.linspace(start = 0, stop = 1, num = 1500)
jnt_motion2_acc = jnt_motion2_acc * (22.5/sum(jnt_motion2_acc))
print("acc: ", sum(jnt_motion2_acc))

jnt_motion2_ss = np.full(1000, 1)   # create an array of 1000 1's
ss = jnt_motion2_acc[-1]
jnt_motion2_ss = jnt_motion2_ss*ss # scales the motion to the last value of acceleration
print(ss)
print("ss: ", sum(jnt_motion2_ss))
print(jnt_motion2_ss[0])

jnt_motion2_dec = np.linspace(start = 1, stop = 0, num = 1500)
jnt_motion2_dec = jnt_motion2_dec * (22.5/sum(jnt_motion2_dec))
print("dec : ", sum(jnt_motion2_dec))

#postive increment from end to center
jnt_motion3_acc = jnt_motion1_acc

jnt_motion3_dec = jnt_motion1_dec

jnt_motion1 = np.append(jnt_motion1_acc, jnt_motion1_dec)
jnt_motion2 = np.append(jnt_motion2_acc, jnt_motion2_dec)
jnt_motion3 = np.append(jnt_motion3_acc, jnt_motion3_dec)

signal_1 = np.append(jnt_motion1, jnt_motion2)
signal_1 = np.append(signal_1, jnt_motion3)

print("signal_1:", sum(signal_1))
signal_1 *= 7.5         #scaling factor
print("signal_1 scaled: ", sum(signal_1))

#signal definition for j3
#positive increment from bottom to  top
jnt_up_acc = np.linspace(start = 0, stop = 1, num = 2000) #0 -> 90
jnt_up_acc = jnt_up_acc * (22.5/sum(jnt_up_acc))

jnt_up_dec = np.linspace(start = 1, stop = 0, num = 2000)  #90-> 180
jnt_up_dec = jnt_up_dec * (22.5/sum(jnt_up_dec))
    #negative increment from top to bottom
jnt_down_acc = jnt_up_acc     #180 -> 270 
jnt_down_dec = jnt_up_dec     #270 -> 360

jnt_up = np.append(jnt_up_acc, jnt_up_dec)
jnt_down = np.append(jnt_down_acc, jnt_down_dec)

signal_3 = np.append(jnt_up, jnt_down)

signal_3 *= 7.5