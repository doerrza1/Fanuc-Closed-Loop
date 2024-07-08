# Function to create signals for Stream Motion given a list of lists 
import numpy as np

def create(lol):
    sig = []
    for i, v_list in enumerate(lol):
        
        if (v_list[1] == 0):
            accel = np.full(1, 0)
            steady_state = np.full(1, 0)
            deccel = np.full(1, 0)
        else:   
            accel = np.linspace(start = 0, stop = v_list[0], num = v_list[1])
            steady_state = np.full(v_list[2], v_list[0])
            deccel = np.flip(accel)

        signal = np.append(accel, steady_state)
        signal = np.append(signal, deccel)

        # Create reverse signal to move robot back to starting loop position
        if (v_list[0] != 0):

            if (input(f"Would you like to reverse motion for axis/joint {i + 1} (y/n)? ").strip().upper() == "Y"):

                reverse = signal*(-1)
                signal = np.append(signal, reverse)
                print("----------------------------")
                
        sig.append(signal)
        

    v = normalize(sig)
    return v

# Function to pad lists with zeroes to ensure no out of range errors occur
def normalize(lol):
    new_lol = []
    # Returns the length of the longest array
    max_len = len(max(lol, key=len))
    # Iterates through the lol for each signal
    for i, sig in enumerate(lol):
        # Checks to see if the signal is shorter than the longest signal
        if (len(sig) < max_len):
            # Obtains the difference in length
            diff = max_len - len(sig)

            # User input for padding before or after motion
            # confirm = input(f"Delay Motion for axis/joint {i + 1} (y/n): ").strip().upper()
            # print("----------------------------")
            # if (confirm == "Y"):
            #     # Pads the array with diff 0s at the beginning of the array
            #     new_sig = np.pad(sig, (diff,0), 'constant')

            # else:
            #     # Pads the array with diff 0s at the end of the array
            #     new_sig = np.pad(sig, (0, diff), 'constant')
            # Appends the new signal to the new lol

            # Removed Padding Functionality for ease of use with vvfull
            new_sig = np.pad(sig, (diff,0), 'constant')
            new_lol.append(new_sig)
        
        else:
            new_lol.append(sig)


    return new_lol