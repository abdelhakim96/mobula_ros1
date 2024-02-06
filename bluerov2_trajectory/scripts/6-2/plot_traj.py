import os
import matplotlib.pyplot as plt
import numpy as np

plt.close('all')

folder_name = ""

# File paths containing the recorded data
file_name_gp_nogp = os.path.join(folder_name, "nogp.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "lambda1.txt")
file_name_gp_lambda08 = os.path.join(folder_name, "lambda08.txt")
file_name_gp_lambda09 = os.path.join(folder_name, "lambda09.txt")
file_name_macgp = os.path.join(folder_name, "macgp_new.txt")
file_name_newest = os.path.join(folder_name, "newest.txt")  # Added line for newest.txt

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

# Lists for newest.txt
ref_positions_x_newest = []
ref_positions_y_newest = []
actual_positions_x_newest = []
actual_positions_y_newest = []

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

# Read data from newest.txt
read_data(file_name_newest, ref_positions_x_newest, ref_positions_y_newest, actual_positions_x_newest, actual_positions_y_newest)

# Check if data is read successfully
print("GP_nogp Data:", len(actual_positions_x_gp_nogp))
print("lambda01 Data:", len(actual_positions_x_gp_lambda01))
print("lambda08 Data:", len(actual_positions_x_gp_lambda08))
print("lambda09 Data:", len(actual_positions_x_gp_lambda09))
print("macgp Data:", len(actual_positions_x_macgp))
print("newest Data:", len(actual_positions_x_newest))  # Added line for newest.txt

# Plotting a subset of 650 points from each trajectory
subset_size = 1300

plt.figure(figsize=(10, 6))

# Plot for GP_nogp
plt.plot(actual_positions_x_gp_nogp[:subset_size], actual_positions_y_gp_nogp[:subset_size], linewidth=2, label='GP_nogp', color='blue')

# Plot for lambda01
plt.plot(actual_positions_x_gp_lambda01[:subset_size], actual_positions_y_gp_lambda01[:subset_size], linewidth=2, label='lambda01', color='orange')

# Plot for lambda08
plt.plot(actual_positions_x_gp_lambda08[:subset_size], actual_positions_y_gp_lambda08[:subset_size], linewidth=2, label='lambda08', color='green')

# Plot for lambda09
#plt.plot(actual_positions_x_gp_lambda09[:subset_size], actual_positions_y_gp_lambda09[:subset_size], linewidth=2, label='lambda09', color='red')

# Plot for macgp
plt.plot(actual_positions_x_macgp[:subset_size], actual_positions_y_macgp[:subset_size], linewidth=2, label='macgp', color='red')

# Plot for newest.txt
plt.plot(actual_positions_x_newest[:subset_size], actual_positions_y_newest[:subset_size], linewidth=2, label='newest', color='brown')  # Added line for newest.txt

# Plot for the unit circle (Reference)
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(np.cos(theta), np.sin(theta), linestyle='--', linewidth=4, label='Reference', color='black')

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
error_newest = calculate_error(actual_positions_x_newest[:subset_size], actual_positions_y_newest[:subset_size], ref_positions_x_newest[:subset_size], ref_positions_y_newest[:subset_size])  # Added line for newest.txt

# Plotting the absolute error against time

mean_error_gp_nogp = np.mean(error_gp_nogp)
mean_error_gp_lambda01 = np.mean(error_gp_lambda01)
mean_error_gp_lambda08 = np.mean(error_gp_lambda08)
mean_error_gp_lambda09 = np.mean(error_gp_lambda09)
mean_error_macgp = np.mean(error_macgp)
mean_error_newest = np.mean(error_newest)

# Plotting the absolute error against time
plt.plot(range(len(error_gp_nogp)), error_gp_nogp, label=f'GP_nogp (Mean Error: {mean_error_gp_nogp:.2f})', color='blue')
plt.plot(range(len(error_gp_lambda01)), error_gp_lambda01, label=f'lambda01 (Mean Error: {mean_error_gp_lambda01:.2f})', color='orange')
plt.plot(range(len(error_gp_lambda08)), error_gp_lambda08, label=f'lambda08 (Mean Error: {mean_error_gp_lambda08:.2f})', color='green')
plt.plot(range(len(error_gp_lambda09)), error_gp_lambda09, label=f'lambda09 (Mean Error: {mean_error_gp_lambda09:.2f})', color='red')
plt.plot(range(len(error_macgp)), error_macgp, label=f'macgp (Mean Error: {mean_error_macgp:.2f})', color='purple')
plt.plot(range(len(error_newest)), error_newest,  linestyle='--', linewidth=2, label=f'newest (Mean Error: {mean_error_newest:.2f})', color='brown')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time for All Methods')
plt.legend()
plt.grid(True)

plt.show()
