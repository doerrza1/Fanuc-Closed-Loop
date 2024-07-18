import numpy as np
from src.plot import *

inc = np.linspace(start = 1, stop = 65, num = 65)
print(len(inc))


sig = 250
acc = 1
acc = acc*8/1000
init_pos = 300


lis1 = []
lis2 = []
for i in inc:

    pos = 0.5*acc*(i**2) + init_pos
    if (i == 64):
        p_2 = pos
    if (i == 65):
        p_1 = pos
        v = p_1 - p_2 # Final velocity
        print("Final Velo: ", v)

    lis1.append(pos)

for i in inc:      

    pos = -0.5*acc*(i**2) + i*v + p_1

    lis2.append(pos)
    
print(lis1)
print(lis2)