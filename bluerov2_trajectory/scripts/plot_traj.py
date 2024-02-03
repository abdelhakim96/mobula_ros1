import os
import matplotlib.pyplot as plt
import numpy as np
folder_name = "3-2"

# File paths containing the recorded data
file_name_gp_fixed_l = os.path.join(folder_name, "recorded_data_gp_lambda_0.97.txt")
file_name_agp = os.path.join(folder_name, "recorded_data_agp.txt")
file_name_nogp = os.path.join(folder_name, "mhe_3W.txt")

# Lists to store the trajectory data
ref_positions_x_gp = []
ref_positions_y_gp = []
actual_positions_x_gp = []
actual_positions_y_gp = []

ref_positions_x_agp = []
ref_positions_y_agp = []
actual_positions_x_agp = []
actual_positions_y_agp = []

ref_positions_x_nogp = []
ref_positions_y_nogp = []
actual_positions_x_nogp = []
actual_positions_y_nogp = []

# Read data from the first file (gp with fixed lambda)
with open(file_name_gp_fixed_l, 'r') as file_gp_fixed_l:
    for line in file_gp_fixed_l:
        data = line.strip().split(',')
        ref_positions_x_gp.append(float(data[0]))
        ref_positions_y_gp.append(float(data[1]))
        actual_positions_x_gp.append(float(data[3]))
        actual_positions_y_gp.append(float(data[4]))

# Read data from the second file (adaptive gp)
with open(file_name_agp, 'r') as file_agp:
    for line in file_agp:
        data = line.strip().split(',')
        ref_positions_x_agp.append(float(data[0]))
        ref_positions_y_agp.append(float(data[1]))
        actual_positions_x_agp.append(float(data[3]))
        actual_positions_y_agp.append(float(data[4]))

# Read data from the third file (no_gp)
with open(file_name_nogp, 'r') as file_nogp:
    for line in file_nogp:
        data = line.strip().split(',')
        ref_positions_x_nogp.append(float(data[0]))
        ref_positions_y_nogp.append(float(data[1]))
        actual_positions_x_nogp.append(float(data[3]))
        actual_positions_y_nogp.append(float(data[4]))

# Plotting a subset of 1000 points from each trajectory
subset_size = 800

plt.figure(figsize=(10, 6))

# Plot for gp with fixed lambda
plt.plot(actual_positions_x_gp[:subset_size], actual_positions_y_gp[:subset_size], label=' (GP Fixed Lambda)', color='blue')

# Plot for adaptive gp
plt.plot(actual_positions_x_agp[:subset_size], actual_positions_y_agp[:subset_size], label='(Adaptive GP)', color='orange')

# Plot for no_gp
plt.plot(ref_positions_x_nogp[:subset_size], ref_positions_y_nogp[:subset_size], label='Reference Trajectory', color='red', linestyle='--')
plt.plot(actual_positions_x_nogp[:subset_size], actual_positions_y_nogp[:subset_size], label=' MHE', color='green')

plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Reference vs Actual Trajectories')
plt.legend()
plt.grid(True)



# Plotting the absolute error against time for all methods
plt.figure(figsize=(10, 6))

# Calculate absolute error for each method
error_gp = np.sqrt((np.array(actual_positions_x_gp[:1000]) - np.array(ref_positions_x_gp[:1000]))**2 +
                   (np.array(actual_positions_y_gp[:1000]) - np.array(ref_positions_y_gp[:1000]))**2)

error_agp = np.sqrt((np.array(actual_positions_x_agp[:1000]) - np.array(ref_positions_x_agp[:1000]))**2 +
                    (np.array(actual_positions_y_agp[:1000]) - np.array(ref_positions_y_agp[:1000]))**2)

error_nogp = np.sqrt((np.array(actual_positions_x_nogp[:1000]) - np.array(ref_positions_x_nogp[:1000]))**2 +
                     (np.array(actual_positions_y_nogp[:1000]) - np.array(ref_positions_y_nogp[:1000]))**2)

# Plotting the absolute error against time
plt.plot(range(len(error_gp)), error_gp, label=' (GP Fixed Lambda)', color='blue')
plt.plot(range(len(error_agp)), error_agp, label=' (Adaptive GP)', color='orange')
plt.plot(range(len(error_nogp)), error_nogp, label='MHE', color='green')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time for All Methods')
plt.legend()
plt.grid(True)

plt.show()
