import os
import matplotlib.pyplot as plt
import numpy as np

plt.close('all')

folder_name = "5-2/dense"

# File paths containing the recorded data
file_name_gp_nogp = os.path.join(folder_name, "nogp.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "lambda1.txt")
file_name_gp_lambda08 = os.path.join(folder_name, "lambda08.txt")
file_name_gp_lambda09 = os.path.join(folder_name, "lambda09.txt")
file_name_macgp = os.path.join(folder_name, "macgp.txt")

# Lists to store the trajectory data
ref_positions_x_gp_nogp = []
ref_positions_y_gp_nogp = []
actual_positions_x_gp_nogp = []
actual_positions_y_gp_nogp = []

ref_positions_x_gp_lambda01 = []
ref_positions_y_gp_lambda01 = []
actual_positions_x_gp_lambda01 = []
actual_positions_y_gp_lambda01 = []

ref_positions_x_gp_lambda08 = []
ref_positions_y_gp_lambda08 = []
actual_positions_x_gp_lambda08 = []
actual_positions_y_gp_lambda08 = []

ref_positions_x_gp_lambda09 = []
ref_positions_y_gp_lambda09 = []
actual_positions_x_gp_lambda09 = []
actual_positions_y_gp_lambda09 = []

ref_positions_x_macgp = []
ref_positions_y_macgp = []
actual_positions_x_macgp = []
actual_positions_y_macgp = []

# Function to read data from a file and populate the lists
def read_data(file_path, ref_x_list, ref_y_list, actual_x_list, actual_y_list):
    try:
        with open(file_path, 'r') as file:
            for line in file:
                data = line.strip().split(',')
                ref_x_list.append(float(data[0]))
                ref_y_list.append(float(data[1]))
                actual_x_list.append(float(data[3]))
                actual_y_list.append(float(data[4]))
    except Exception as e:
        print(f"Error reading data from {file_path}: {e}")

# Read data from files
read_data(file_name_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp)
read_data(file_name_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01)
read_data(file_name_gp_lambda08, ref_positions_x_gp_lambda08, ref_positions_y_gp_lambda08, actual_positions_x_gp_lambda08, actual_positions_y_gp_lambda08)
read_data(file_name_gp_lambda09, ref_positions_x_gp_lambda09, ref_positions_y_gp_lambda09, actual_positions_x_gp_lambda09, actual_positions_y_gp_lambda09)
read_data(file_name_macgp, ref_positions_x_macgp, ref_positions_y_macgp, actual_positions_x_macgp, actual_positions_y_macgp)

# Check if data is read successfully
print("GP_nogp Data:", len(actual_positions_x_gp_nogp))
print("lambda01 Data:", len(actual_positions_x_gp_lambda01))
print("lambda08 Data:", len(actual_positions_x_gp_lambda08))
print("lambda09 Data:", len(actual_positions_x_gp_lambda09))
print("macgp Data:", len(actual_positions_x_macgp))

# Plotting a subset of 700 points from each trajectory
subset_size = 700

plt.figure(figsize=(10, 6))

# Plot for GP_nogp
plt.plot(actual_positions_x_gp_nogp[:subset_size], actual_positions_y_gp_nogp[:subset_size], label='GP_nogp', color='blue')

# Plot for lambda01
plt.plot(actual_positions_x_gp_lambda01[:subset_size], actual_positions_y_gp_lambda01[:subset_size], label='lambda01', color='orange')

# Plot for lambda08
plt.plot(actual_positions_x_gp_lambda08[:subset_size], actual_positions_y_gp_lambda08[:subset_size], label='lambda08', color='green')

# Plot for lambda09
plt.plot(actual_positions_x_gp_lambda09[:subset_size], actual_positions_y_gp_lambda09[:subset_size], label='lambda09', color='red')

# Plot for macgp
plt.plot(actual_positions_x_macgp[:subset_size], actual_positions_y_macgp[:subset_size], label='macgp', color='purple')

plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Reference vs Actual Trajectories')
plt.legend()
plt.grid(True)

# Plotting the absolute error against time for all methods
plt.figure(figsize=(10, 6))

# Function to calculate absolute error
def calculate_error(actual_x, actual_y, ref_x, ref_y):
    return np.sqrt((np.array(actual_x) - np.array(ref_x))**2 + (np.array(actual_y) - np.array(ref_y))**2)

# Calculate absolute error for each method
error_gp_nogp = calculate_error(actual_positions_x_gp_nogp[:subset_size], actual_positions_y_gp_nogp[:subset_size], ref_positions_x_gp_nogp[:subset_size], ref_positions_y_gp_nogp[:subset_size])
error_gp_lambda01 = calculate_error(actual_positions_x_gp_lambda01[:subset_size], actual_positions_y_gp_lambda01[:subset_size], ref_positions_x_gp_lambda01[:subset_size], ref_positions_y_gp_lambda01[:subset_size])
error_gp_lambda08 = calculate_error(actual_positions_x_gp_lambda08[:subset_size], actual_positions_y_gp_lambda08[:subset_size], ref_positions_x_gp_lambda08[:subset_size], ref_positions_y_gp_lambda08[:subset_size])
error_gp_lambda09 = calculate_error(actual_positions_x_gp_lambda09[:subset_size], actual_positions_y_gp_lambda09[:subset_size], ref_positions_x_gp_lambda09[:subset_size], ref_positions_y_gp_lambda09[:subset_size])
error_macgp = calculate_error(actual_positions_x_macgp[:subset_size], actual_positions_y_macgp[:subset_size], ref_positions_x_macgp[:subset_size], ref_positions_y_macgp[:subset_size])

# Plotting the absolute error against time
plt.plot(range(len(error_gp_nogp)), error_gp_nogp, label='GP_nogp', color='blue')
plt.plot(range(len(error_gp_lambda01)), error_gp_lambda01, label='lambda01', color='orange')
plt.plot(range(len(error_gp_lambda08)), error_gp_lambda08, label='lambda08', color='green')
plt.plot(range(len(error_gp_lambda09)), error_gp_lambda09, label='lambda09', color='red')
plt.plot(range(len(error_macgp)), error_macgp, label='macgp', color='purple')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time for All Methods')
plt.legend()
plt.grid(True)

plt.show()
