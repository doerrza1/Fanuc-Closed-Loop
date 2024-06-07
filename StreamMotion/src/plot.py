# Function for plotting numpy arrays using matplotlib

import numpy as np
import matplotlib.pyplot as plt



def plot(y, x = np.linspace(0, 8000, 8000)):
    
    # Plot
    plt.plot(x, y)

    #Labels
    plt.xlabel("Pack Number")
    plt.ylabel("Signal")
    plt.title("Plot of Signal")

    plt.show()




    
