import os
import matplotlib.pyplot as plt
import numpy as np

plt.close('all')

folder_name = ""

# Corrected file paths containing the recorded data
file_name_gp_nogp = os.path.join(folder_name, "no_gp.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "lambda1.txt")
file_name_gp_lambda01_combined = os.path.join(folder_name, "lambda1_combined.txt")
file_name_gp_lambda09 = os.path.join(folder_name, "lambda9.txt")
file_name_gp_lambda09_combined = os.path.join(folder_name, "lambda9_combined.txt")

# Lists to store the trajectory data
ref_positions_x_gp_nogp = []
ref_positions_y_gp_nogp = []
actual_positions_x_gp_nogp = []
actual_positions_y_gp_nogp = []

ref_positions_x_gp_lambda01 = []
ref_positions_y_gp_lambda01 = []
actual_positions_x_gp_lambda01 = []
actual_positions_y_gp_lambda01 = []

ref_positions_x_gp_lambda01_combined = []
ref_positions_y_gp_lambda01_combined = []
actual_positions_x_gp_lambda01_combined = []
actual_positions_y_gp_lambda01_combined = []

ref_positions_x_gp_lambda09 = []
ref_positions_y_gp_lambda09 = []
actual_positions_x_gp_lambda09 = []
actual_positions_y_gp_lambda09 = []

ref_positions_x_gp_lambda09_combined = []
ref_positions_y_gp_lambda09_combined = []
actual_positions_x_gp_lambda09_combined = []
actual_positions_y_gp_lambda09_combined = []

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
read_data(file_name_gp_lambda01_combined, ref_positions_x_gp_lambda01_combined, ref_positions_y_gp_lambda01_combined, actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined)
read_data(file_name_gp_lambda09, ref_positions_x_gp_lambda09, ref_positions_y_gp_lambda09, actual_positions_x_gp_lambda09, actual_positions_y_gp_lambda09)
read_data(file_name_gp_lambda09_combined, ref_positions_x_gp_lambda09_combined, ref_positions_y_gp_lambda09_combined, actual_positions_x_gp_lambda09_combined, actual_positions_y_gp_lambda09_combined)

# Check if data is read successfully
print("GP_nogp Data:", len(actual_positions_x_gp_nogp))
print("lambda01 Data:", len(actual_positions_x_gp_lambda01))
print("lambda1_combined Data:", len(actual_positions_x_gp_lambda01_combined))
print("lambda09 Data:", len(actual_positions_x_gp_lambda09))
print("lambda9_combined Data:", len(actual_positions_x_gp_lambda09_combined))

# Calculate absolute error
def calculate_error(actual_x, actual_y, ref_x, ref_y):
    return np.sqrt((np.array(actual_x) - np.array(ref_x))**2 + (np.array(actual_y) - np.array(ref_y))**2)

error_gp_nogp = calculate_error(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp)
error_gp_lambda01 = calculate_error(actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01)
error_gp_lambda01_combined = calculate_error(actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined, ref_positions_x_gp_lambda01_combined, ref_positions_y_gp_lambda01_combined)
error_gp_lambda09 = calculate_error(actual_positions_x_gp_lambda09, actual_positions_y_gp_lambda09, ref_positions_x_gp_lambda09, ref_positions_y_gp_lambda09)
error_gp_lambda09_combined = calculate_error(actual_positions_x_gp_lambda09_combined, actual_positions_y_gp_lambda09_combined, ref_positions_x_gp_lambda09_combined, ref_positions_y_gp_lambda09_combined)

# Calculate mean error
mean_error_gp_nogp = np.mean(error_gp_nogp)
mean_error_gp_lambda01 = np.mean(error_gp_lambda01)
mean_error_gp_lambda01_combined = np.mean(error_gp_lambda01_combined)
mean_error_gp_lambda09 = np.mean(error_gp_lambda09)
mean_error_gp_lambda09_combined = np.mean(error_gp_lambda09_combined)

# Plotting a subset of 650 points from each trajectory
subset_size = 1200

plt.figure(figsize=(10, 6))

# Plot for GP_nogp
plt.plot(actual_positions_x_gp_nogp[:subset_size], actual_positions_y_gp_nogp[:subset_size], linewidth=2, label=f'GP_nogp (Mean Error: {mean_error_gp_nogp:.2f})', color='blue')

# Plot for lambda01
plt.plot(actual_positions_x_gp_lambda01[:subset_size], actual_positions_y_gp_lambda01[:subset_size], linewidth=2, label=f'lambda01 (Mean Error: {mean_error_gp_lambda01:.2f})', color='orange')

# Plot for lambda1_combined
plt.plot(actual_positions_x_gp_lambda01_combined[:subset_size], actual_positions_y_gp_lambda01_combined[:subset_size], linewidth=2, label=f'lambda1_combined (Mean Error: {mean_error_gp_lambda01_combined:.2f})', color='purple')

# Plot for lambda09
plt.plot(actual_positions_x_gp_lambda09[:subset_size], actual_positions_y_gp_lambda09[:subset_size], linewidth=2, label=f'lambda09 (Mean Error: {mean_error_gp_lambda09:.2f})', color='green')

# Plot for lambda9_combined
plt.plot(actual_positions_x_gp_lambda09_combined[:subset_size], actual_positions_y_gp_lambda09_combined[:subset_size], linewidth=2, label=f'lambda9_combined (Mean Error: {mean_error_gp_lambda09_combined:.2f})', color='red')

# Plot for the unit circle (Reference)
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(1 + np.cos(theta), np.sin(theta), linestyle='--', linewidth=4, label='Reference', color='black')

plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Reference vs Actual Trajectories')
plt.legend()
plt.grid(True)

# Plotting the absolute error
plt.figure(figsize=(10, 6))

plt.plot(range(len(error_gp_nogp[:subset_size])), error_gp_nogp[:subset_size], label=f'GP_nogp (Mean Error: {mean_error_gp_nogp:.2f})', color='blue')
plt.plot(range(len(error_gp_lambda01[:subset_size])), error_gp_lambda01[:subset_size], label=f'lambda01 (Mean Error: {mean_error_gp_lambda01:.2f})', color='orange')
plt.plot(range(len(error_gp_lambda01_combined[:subset_size])), error_gp_lambda01_combined[:subset_size], label=f'lambda1_combined (Mean Error: {mean_error_gp_lambda01_combined:.2f})', color='purple')
plt.plot(range(len(error_gp_lambda09[:subset_size])), error_gp_lambda09[:subset_size], label=f'lambda09 (Mean Error: {mean_error_gp_lambda09:.2f})', color='green')
plt.plot(range(len(error_gp_lambda09_combined[:subset_size])), error_gp_lambda09_combined[:subset_size], label=f'lambda9_combined (Mean Error: {mean_error_gp_lambda09_combined:.2f})', color='red')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time')
plt.legend()
plt.grid(True)

plt.show()
