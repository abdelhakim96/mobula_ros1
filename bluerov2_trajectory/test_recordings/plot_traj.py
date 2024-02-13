import os
import matplotlib.pyplot as plt
import numpy as np

plt.close('all')

folder_name = ""

# Corrected file paths containing the recorded data
file_name_gp_nogp = os.path.join(folder_name, "nogp_test.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "lambda1_sine.txt")
file_name_gp_lambda01_combined = os.path.join(folder_name, "lambda1_test.txt")
file_name_gp_lambda09 = os.path.join(folder_name, "lambda8_sine.txt")
file_name_gp_lambda09_combined = os.path.join(folder_name, "lambda8_test.txt")
file_name_gp_mac_sine = os.path.join(folder_name, "mac_sine.txt")
file_name_gp_mac_test = os.path.join(folder_name, "mac_test.txt")

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

ref_positions_x_gp_mac_sine = []
ref_positions_y_gp_mac_sine = []
actual_positions_x_gp_mac_sine = []
actual_positions_y_gp_mac_sine = []

ref_positions_x_gp_mac_test = []
ref_positions_y_gp_mac_test = []
actual_positions_x_gp_mac_test = []
actual_positions_y_gp_mac_test = []

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
read_data(file_name_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine)
read_data(file_name_gp_mac_test, ref_positions_x_gp_mac_test, ref_positions_y_gp_mac_test, actual_positions_x_gp_mac_test, actual_positions_y_gp_mac_test)

# Check if data is read successfully
print("GP_nogp Data:", len(actual_positions_x_gp_nogp))
print("lambda01 Data:", len(actual_positions_x_gp_lambda01))
print("lambda1_combined Data:", len(actual_positions_x_gp_lambda01_combined))
print("lambda09 Data:", len(actual_positions_x_gp_lambda09))
print("lambda9_combined Data:", len(actual_positions_x_gp_lambda09_combined))
print("mac_sine Data:", len(actual_positions_x_gp_mac_sine))
print("mac_test Data:", len(actual_positions_x_gp_mac_test))

# Calculate absolute error for each dataset
def calculate_error(actual_x, actual_y, ref_x, ref_y):
    return np.sqrt((np.array(actual_x) - np.array(ref_x))**2 + (np.array(actual_y) - np.array(ref_y))**2)

error_gp_nogp = calculate_error(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp)
error_gp_lambda01 = calculate_error(actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01)
error_gp_lambda01_combined = calculate_error(actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined, ref_positions_x_gp_lambda01_combined, ref_positions_y_gp_lambda01_combined)
error_gp_lambda09 = calculate_error(actual_positions_x_gp_lambda09, actual_positions_y_gp_lambda09, ref_positions_x_gp_lambda09, ref_positions_y_gp_lambda09)
error_gp_lambda09_combined = calculate_error(actual_positions_x_gp_lambda09_combined, actual_positions_y_gp_lambda09_combined, ref_positions_x_gp_lambda09_combined, ref_positions_y_gp_lambda09_combined)
error_gp_mac_sine = calculate_error(actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine)
error_gp_mac_test = calculate_error(actual_positions_x_gp_mac_test, actual_positions_y_gp_mac_test, ref_positions_x_gp_mac_test, ref_positions_y_gp_mac_test)

# Calculate mean error for each dataset
mean_error_gp_nogp = np.mean(error_gp_nogp)
mean_error_gp_lambda01 = np.mean(error_gp_lambda01)
mean_error_gp_lambda01_combined = np.mean(error_gp_lambda01_combined)
mean_error_gp_lambda09 = np.mean(error_gp_lambda09)
mean_error_gp_lambda09_combined = np.mean(error_gp_lambda09_combined)
mean_error_gp_mac_sine = np.mean(error_gp_mac_sine)
mean_error_gp_mac_test = np.mean(error_gp_mac_test)

# Function to calculate error as the average closest perpendicular distance to the reference circle
def calculate_perpendicular_error(actual_x, actual_y, ref_x, ref_y):
    errors = []
    for i in range(len(actual_x)):
        # Calculate the closest point on the reference circle to the actual point
        ref_circle_x = 1 + np.cos(ref_x[i])
        ref_circle_y = np.sin(ref_y[i])
        actual_point = np.array([actual_x[i], actual_y[i]])
        ref_circle_point = np.array([ref_circle_x, ref_circle_y])
        ref_to_actual_vector = actual_point - ref_circle_point
        ref_to_circle_vector = ref_circle_point - np.array([1, 0])
        perpendicular_vector = np.cross(ref_to_actual_vector, ref_to_circle_vector)
        perpendicular_distance = np.linalg.norm(perpendicular_vector) / np.linalg.norm(ref_to_circle_vector)
        errors.append(perpendicular_distance)
    return np.mean(errors)

# Calculate perpendicular error for each dataset
perpendicular_error_gp_nogp = calculate_perpendicular_error(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp)
perpendicular_error_gp_lambda01 = calculate_perpendicular_error(actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01)
perpendicular_error_gp_lambda01_combined = calculate_perpendicular_error(actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined, ref_positions_x_gp_lambda01_combined, ref_positions_y_gp_lambda01_combined)
perpendicular_error_gp_lambda09 = calculate_perpendicular_error(actual_positions_x_gp_lambda09, actual_positions_y_gp_lambda09, ref_positions_x_gp_lambda09, ref_positions_y_gp_lambda09)
perpendicular_error_gp_lambda09_combined = calculate_perpendicular_error(actual_positions_x_gp_lambda09_combined, actual_positions_y_gp_lambda09_combined, ref_positions_x_gp_lambda09_combined, ref_positions_y_gp_lambda09_combined)
perpendicular_error_gp_mac_sine = calculate_perpendicular_error(actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine)
perpendicular_error_gp_mac_test = calculate_perpendicular_error(actual_positions_x_gp_mac_test, actual_positions_y_gp_mac_test, ref_positions_x_gp_mac_test, ref_positions_y_gp_mac_test)

# Print the perpendicular errors
print("Perpendicular Error GP_nogp:", perpendicular_error_gp_nogp)
print("Perpendicular Error lambda01:", perpendicular_error_gp_lambda01)
print("Perpendicular Error lambda1_combined:", perpendicular_error_gp_lambda01_combined)
print("Perpendicular Error lambda09:", perpendicular_error_gp_lambda09)
print("Perpendicular Error lambda9_combined:", perpendicular_error_gp_lambda09_combined)
print("Perpendicular Error mac_sine:", perpendicular_error_gp_mac_sine)
print("Perpendicular Error mac_test:", perpendicular_error_gp_mac_test)

# Plotting a subset of 650 points from each trajectory
subset_size = 650

plt.figure(figsize=(10, 6))

# Plot for GP_nogp
plt.plot(actual_positions_x_gp_nogp[:subset_size], actual_positions_y_gp_nogp[:subset_size], linewidth=2, label=f'GP_nogp (Mean Error: {mean_error_gp_nogp:.2f})', color='blue')

# Plot for lambda01
plt.plot(actual_positions_x_gp_lambda01[:subset_size], actual_positions_y_gp_lambda01[:subset_size], linewidth=3, label=f'lambda01 (Mean Error: {mean_error_gp_lambda01:.2f})', color='yellow')

# Plot for lambda1_combined
plt.plot(actual_positions_x_gp_lambda01_combined[:subset_size], actual_positions_y_gp_lambda01_combined[:subset_size], linewidth=2, label=f'lambda1_combined (Mean Error: {mean_error_gp_lambda01_combined:.2f})', color='purple')

# Plot for lambda09
plt.plot(actual_positions_x_gp_lambda09[:subset_size], actual_positions_y_gp_lambda09[:subset_size], linewidth=2, label=f'lambda09 (Mean Error: {mean_error_gp_lambda09:.2f})', color='green')

# Plot for lambda9_combined
plt.plot(actual_positions_x_gp_lambda09_combined[:subset_size], actual_positions_y_gp_lambda09_combined[:subset_size], linewidth=2, label=f'lambda9_combined (Mean Error: {mean_error_gp_lambda09_combined:.2f})', color='red')

# Plot for mac_sine
plt.plot(actual_positions_x_gp_mac_sine[:subset_size], actual_positions_y_gp_mac_sine[:subset_size], linewidth=2, label=f'mac_sine (Mean Error: {mean_error_gp_mac_sine:.2f})', color='cyan')

# Plot for mac_test
plt.plot(actual_positions_x_gp_mac_test[:subset_size], actual_positions_y_gp_mac_test[:subset_size], linewidth=2, label=f'mac_test (Mean Error: {mean_error_gp_mac_test:.2f})', color='magenta')

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
plt.plot(range(len(error_gp_nogp[:subset_size])), error_gp_nogp[:subset_size], label=f'GP_nogp (Mean Error: {mean_error_gp_nogp:.2g})', color='blue')
plt.plot(range(len(error_gp_lambda01[:subset_size])), error_gp_lambda01[:subset_size], label=f'lambda01 (Mean Error: {mean_error_gp_lambda01:.2g})', color='orange')
plt.plot(range(len(error_gp_lambda01_combined[:subset_size])), error_gp_lambda01_combined[:subset_size], label=f'lambda1_combined (Mean Error: {mean_error_gp_lambda01_combined:.2g})', color='purple')
plt.plot(range(len(error_gp_lambda09[:subset_size])), error_gp_lambda09[:subset_size], label=f'lambda09 (Mean Error: {mean_error_gp_lambda09:.2g})', color='green')
plt.plot(range(len(error_gp_lambda09_combined[:subset_size])), error_gp_lambda09_combined[:subset_size], label=f'lambda9_combined (Mean Error: {mean_error_gp_lambda09_combined:.2g})', color='red')
plt.plot(range(len(error_gp_mac_sine[:subset_size])), error_gp_mac_sine[:subset_size], label=f'mac_sine (Mean Error: {mean_error_gp_mac_sine:.2g})', color='cyan')
plt.plot(range(len(error_gp_mac_test[:subset_size])), error_gp_mac_test[:subset_size], label=f'mac_test (Mean Error: {mean_error_gp_mac_test:.2g})', color='magenta')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time')
plt.legend()
plt.grid(True)

plt.show()
