#Joint Stram Motion Test for Fanuc LR Mate 200iD

import numpy as np
from src.plot import *
import matplotlib

matplotlib.use('TkAgg')
# signal definition for j1  (0 -> 45 -> 0 -> -45 -> 0)

#using linspace creates an array of numbers evenly spaced between start and stop
#causes the robot to accelerate (start = 0, stop = 45) or decelerate(start = 45, stop = 0)
#by incrementing by a higher (acc) or lower (dec)

#positive increment from center to end
jnt_motion1_acc = np.linspace(start = 0, stop = 1, num = 1000)
jnt_motion1_acc = jnt_motion1_acc *(22.5/sum(jnt_motion1_acc))

jnt_motion1_dec = np.linspace(start = 1, stop = 0, num = 1000)
jnt_motion1_dec = jnt_motion1_dec * (22.5/sum(jnt_motion1_dec))

print(sum(jnt_motion1_acc))
print(sum(jnt_motion1_dec))

    #negative increment from end to end
jnt_motion2_acc = np.linspace(start = 0, stop = 1, num = 1000)
jnt_motion2_acc = jnt_motion2_acc * (22.5/sum(jnt_motion2_acc))

print("last_value: ", jnt_motion2_acc[-1])
steadystate = jnt_motion2_acc[-1]
jnt_motion2_ss = np.full(1000, 1)  # fils array with 1000 1's

# scales steady state motion to the same as final accleration value
jnt_motion2_ss = jnt_motion2_ss*(steadystate)
print("steadystate step: ", jnt_motion2_ss[0])
jnt_motion2_dec = np.linspace(start = 1, stop = 0, num = 1000)
jnt_motion2_dec = jnt_motion2_dec * (22.5/sum(jnt_motion2_dec))

print("acc: ", sum(jnt_motion2_acc))
print("ss: ", sum(jnt_motion2_ss))
print("dec :", sum(jnt_motion2_dec))
    #postive increment from end to center

jnt_motion3_acc = jnt_motion1_acc

jnt_motion3_dec = jnt_motion1_dec

jnt_motion1 = np.append(jnt_motion1_acc, jnt_motion1_dec)
print("motion1: ", sum(jnt_motion1)*2/5)
jnt_motion2 = np.append(jnt_motion2_acc, jnt_motion2_ss)
jnt_motion2 = np.append(jnt_motion2, jnt_motion2_dec)
jnt_motion3 = np.append(jnt_motion3_acc, jnt_motion3_dec)

signal_1 = np.append(jnt_motion1, jnt_motion2)
signal_1 = np.append(signal_1, jnt_motion3)

signal_1 = signal_1*7.5
signal_1 = [round(num, 4) for num in signal_1]
print("signal 1: ", sum(signal_1))


    
#signal definition for j3
    #positive increment from bottom to  top
jnt_up_acc = np.linspace(start = 0, stop = 1, num = 2000) #0 -> 90
jnt_up_acc = jnt_up_acc * (45/sum(jnt_up_acc))


jnt_up_dec = np.linspace(start = 1, stop = 0, num = 2000)  #90-> 180
jnt_up_dec = jnt_up_dec * (45/sum(jnt_up_dec))
    #negative increment from top to bottom
jnt_down_acc = jnt_up_acc     #180 -> 270 
jnt_down_dec = jnt_up_dec     #270 -> 360

jnt_up = np.append(jnt_up_acc, jnt_up_dec)
jnt_down = np.append(jnt_down_acc, jnt_down_dec)

print("up =", sum(jnt_up))
print(sum(jnt_down))
signal_3 = np.append(jnt_up, jnt_down)
signal_3 = signal_3/2

print(sum(signal_3))

for i, value in enumerate(signal_1):
    if (i%100 == 0):
        print(value)

# plot(signal_1)
# plot(signal_3)