import os
import matplotlib.pyplot as plt
import numpy as np

plt.close('all')

folder_name = ""

# Corrected file paths containing the recorded data
file_name_gp_nogp = os.path.join(folder_name, "nogp_sine.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "lambda1_sine.txt")
file_name_gp_lambda01_combined = os.path.join(folder_name, "lambda1_test.txt")
file_name_gp_lambda8 = os.path.join(folder_name, "lambda8_sine.txt")
file_name_gp_lambda8_combined = os.path.join(folder_name, "lambda8_test.txt")
file_name_gp_mac_sine = os.path.join(folder_name, "mac_sine.txt")
file_name_gp_mac_test = os.path.join(folder_name, "mac_test.txt")
file_name_gp_lambda9_sine = os.path.join(folder_name, "lambda9_sine.txt")
file_name_gp_lambda9_test = os.path.join(folder_name, "lambda9_test.txt")
file_name_nogp_test = os.path.join(folder_name, "nogp_test.txt")

# Lists to store the trajectory data
ref_positions_x_gp_nogp = []
ref_positions_y_gp_nogp = []
actual_positions_x_gp_nogp = []
actual_positions_y_gp_nogp = []

ref_positions_x_gp_nogp_test = []
ref_positions_y_gp_nogp_test = []
actual_positions_x_gp_nogp_test = []
actual_positions_y_gp_nogp_test = []

ref_positions_x_gp_lambda01 = []
ref_positions_y_gp_lambda01 = []
actual_positions_x_gp_lambda01 = []
actual_positions_y_gp_lambda01 = []

ref_positions_x_gp_lambda01_combined = []
ref_positions_y_gp_lambda01_combined = []
actual_positions_x_gp_lambda01_combined = []
actual_positions_y_gp_lambda01_combined = []

ref_positions_x_gp_lambda8 = []
ref_positions_y_gp_lambda8 = []
actual_positions_x_gp_lambda8 = []
actual_positions_y_gp_lambda8 = []

ref_positions_x_gp_lambda8_combined = []
ref_positions_y_gp_lambda8_combined = []
actual_positions_x_gp_lambda8_combined = []
actual_positions_y_gp_lambda8_combined = []

ref_positions_x_gp_mac_sine = []
ref_positions_y_gp_mac_sine = []
actual_positions_x_gp_mac_sine = []
actual_positions_y_gp_mac_sine = []

ref_positions_x_gp_mac_test = []
ref_positions_y_gp_mac_test = []
actual_positions_x_gp_mac_test = []
actual_positions_y_gp_mac_test = []

ref_positions_x_gp_lambda9_sine = []
ref_positions_y_gp_lambda9_sine = []
actual_positions_x_gp_lambda9_sine = []
actual_positions_y_gp_lambda9_sine = []

ref_positions_x_gp_lambda9_test = []
ref_positions_y_gp_lambda9_test = []
actual_positions_x_gp_lambda9_test = []
actual_positions_y_gp_lambda9_test = []

# Lists to store the trajectory data




W1 = [] 
W2 = []
W3 =[] 
mu_y =[]
gt_y =[] 

W1_gp_nogp = [] 
W2_gp_nogp = []
W3_gp_nogp = []
mu_y_gp_nogp = []
gt_y_gp_nogp = []

W1_gp_lambda01 = [] 
W2_gp_lambda01 = []
W3_gp_lambda01 = []
mu_y_gp_lambda01 = []
gt_y_gp_lambda01 = []

W1_gp_lambda01_combined = [] 
W2_gp_lambda01_combined = []
W3_gp_lambda01_combined = []
mu_y_gp_lambda01_combined = []
gt_y_gp_lambda01_combined = []

W1_gp_lambda8 = [] 
W2_gp_lambda8 = []
W3_gp_lambda8 = []
mu_y_gp_lambda8 = []
gt_y_gp_lambda8 = []

W1_gp_lambda8_combined = [] 
W2_gp_lambda8_combined = []
W3_gp_lambda8_combined = []
mu_y_gp_lambda8_combined = []
gt_y_gp_lambda8_combined = []

W1_gp_mac_sine = [] 
W2_gp_mac_sine = []
W3_gp_mac_sine = []
mu_y_gp_mac_sine = []
gt_y_gp_mac_sine = []

W1_gp_mac_test = [] 
W2_gp_mac_test = []
W3_gp_mac_test = []
mu_y_gp_mac_test = []
gt_y_gp_mac_test = []

W1_gp_lambda9_sine = [] 
W2_gp_lambda9_sine = []
W3_gp_lambda9_sine = []
mu_y_gp_lambda9_sine = []
gt_y_gp_lambda9_sine = []

W1_gp_lambda9_test = [] 
W2_gp_lambda9_test = []
W3_gp_lambda9_test = []
mu_y_gp_lambda9_test = []
gt_y_gp_lambda9_test = []

# Function to read data from a file and populate the lists
def read_data(file_path, ref_x_list, ref_y_list, actual_x_list, actual_y_list, W1_list, W2_list, W3_list, mu_y_list, gt_y_list):
    try:
        with open(file_path, 'r') as file:
            for line in file:
                data = line.strip().split(',')
                ref_x_list.append(float(data[0]))
                ref_y_list.append(float(data[1]))
                actual_x_list.append(float(data[3]))
                actual_y_list.append(float(data[4]))
                W1_list.append(float(data[17]))
                W2_list.append(float(data[18]))
                W3_list.append(float(data[19]))
                mu_y_list.append(float(data[21]))
                gt_y_list.append(float(data[23]))
                
    except Exception as e:
        print(f"Error reading data from {file_path}: {e}")

# Read data from files
read_data(file_name_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_lambda01_combined, ref_positions_x_gp_lambda01_combined, ref_positions_y_gp_lambda01_combined, actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_lambda8, ref_positions_x_gp_lambda8, ref_positions_y_gp_lambda8, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_lambda8_combined, ref_positions_x_gp_lambda8_combined, ref_positions_y_gp_lambda8_combined, actual_positions_x_gp_lambda8_combined, actual_positions_y_gp_lambda8_combined, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_mac_test, ref_positions_x_gp_mac_test, ref_positions_y_gp_mac_test, actual_positions_x_gp_mac_test, actual_positions_y_gp_mac_test, W1_gp_mac_test, W2_gp_mac_test, W3_gp_mac_test, mu_y_gp_mac_test, gt_y_gp_mac_test)
read_data(file_name_gp_lambda9_sine, ref_positions_x_gp_lambda9_sine, ref_positions_y_gp_lambda9_sine, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, W1, W2, W3, mu_y, gt_y)
read_data(file_name_gp_lambda9_test, ref_positions_x_gp_lambda9_test, ref_positions_y_gp_lambda9_test, actual_positions_x_gp_lambda9_test, actual_positions_y_gp_lambda9_test, W1, W2, W3, mu_y, gt_y)
read_data(file_name_nogp_test, ref_positions_x_gp_nogp_test, ref_positions_y_gp_nogp_test, actual_positions_x_gp_nogp_test, actual_positions_y_gp_nogp_test, W1, W2, W3, mu_y, gt_y)

# Check if data is read successfully
print("GP_nogp Data:", len(actual_positions_x_gp_nogp))
print("lambda01 Data:", len(actual_positions_x_gp_lambda01))
print("lambda1_combined Data:", len(actual_positions_x_gp_lambda01_combined))
print("lambda8 Data:", len(actual_positions_x_gp_lambda8))
print("lambda8_combined Data:", len(actual_positions_x_gp_lambda8_combined))
print("mac_sine Data:", len(actual_positions_x_gp_mac_sine))
print("mac_test Data:", len(actual_positions_x_gp_mac_test))
print("lambda9_sine Data:", len(actual_positions_x_gp_lambda9_sine))
print("lambda9_test Data:", len(actual_positions_x_gp_lambda9_test))

# Calculate absolute error for each dataset
def calculate_error(actual_x, actual_y, ref_x, ref_y):
    return np.sqrt((np.array(actual_x) - np.array(ref_x))**2 + (np.array(actual_y) - np.array(ref_y))**2)

error_gp_nogp = calculate_error(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp)
error_gp_lambda01 = calculate_error(actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01)
error_gp_lambda01_combined = calculate_error(actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined, ref_positions_x_gp_lambda01_combined, ref_positions_y_gp_lambda01_combined)
error_gp_lambda8 = calculate_error(actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, ref_positions_x_gp_lambda8, ref_positions_y_gp_lambda8)
error_gp_lambda8_combined = calculate_error(actual_positions_x_gp_lambda8_combined, actual_positions_y_gp_lambda8_combined, ref_positions_x_gp_lambda8_combined, ref_positions_y_gp_lambda8_combined)
error_gp_mac_sine = calculate_error(actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine)
error_gp_mac_test = calculate_error(actual_positions_x_gp_mac_test, actual_positions_y_gp_mac_test, ref_positions_x_gp_mac_test, ref_positions_y_gp_mac_test)
error_gp_lambda9_sine = calculate_error(actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, ref_positions_x_gp_lambda9_sine, ref_positions_y_gp_lambda9_sine)
error_gp_lambda9_test = calculate_error(actual_positions_x_gp_lambda9_test, actual_positions_y_gp_lambda9_test, ref_positions_x_gp_lambda9_test, ref_positions_y_gp_lambda9_test)

# Calculate mean error for each dataset
mean_error_gp_nogp = np.mean(error_gp_nogp)
mean_error_gp_lambda01 = np.mean(error_gp_lambda01)
mean_error_gp_lambda01_combined = np.mean(error_gp_lambda01_combined)
mean_error_gp_lambda8 = np.mean(error_gp_lambda8)
mean_error_gp_lambda8_combined = np.mean(error_gp_lambda8_combined)
mean_error_gp_mac_sine = np.mean(error_gp_mac_sine)
mean_error_gp_mac_test = np.mean(error_gp_mac_test)
mean_error_gp_lambda9_sine = np.mean(error_gp_lambda9_sine)
mean_error_gp_lambda9_test = np.mean(error_gp_lambda9_test)

# Plotting a subset of 650 points from each trajectory
subset_size = 650

# Plotting the absolute error
plt.figure(figsize=(12, 6))

# Plot for "sine" datasets
plt.subplot(1, 2, 1)

plt.plot(range(len(error_gp_nogp[:subset_size])), error_gp_nogp[:subset_size], label=f'GP_nogp_test (Mean Error: {mean_error_gp_nogp:.2g})', color='blue')

plt.plot(range(len(error_gp_lambda01[:subset_size])), error_gp_lambda01[:subset_size], label=f'lambda01 (Mean Error: {mean_error_gp_lambda01:.2g})', color='orange')
plt.plot(range(len(error_gp_lambda8[:subset_size])), error_gp_lambda8[:subset_size], label=f'lambda8 (Mean Error: {mean_error_gp_lambda8:.2g})', color='green')
plt.plot(range(len(error_gp_mac_sine[:subset_size])), error_gp_mac_sine[:subset_size], label=f'mac_sine (Mean Error: {mean_error_gp_mac_sine:.2g})', color='cyan')
plt.plot(range(len(error_gp_lambda9_sine[:subset_size])), error_gp_lambda9_sine[:subset_size], label=f'lambda9_sine (Mean Error: {mean_error_gp_lambda9_sine:.2g})', color='orange')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time for "Sine" Datasets')
plt.legend()
plt.grid(True)

# Plot for "test" datasets
plt.subplot(1, 2, 2)
#plt.plot(range(len(error_gp_nogp_test[:subset_size])), error_gp_nogp_test[:subset_size], label=f' no gp  (Mean Error: {mean_error_gp_nogp:.2g})', color='blue')
plt.plot(range(len(error_gp_mac_test[:subset_size])), error_gp_mac_test[:subset_size], label=f'mac_test (Mean Error: {mean_error_gp_mac_test:.2g})', color='magenta')
plt.plot(range(len(error_gp_lambda01_combined[:subset_size])), error_gp_lambda01_combined[:subset_size], label=f'lambda1_test (Mean Error: {mean_error_gp_lambda01_combined:.2g})', color='purple')
plt.plot(range(len(error_gp_lambda8_combined[:subset_size])), error_gp_lambda8_combined[:subset_size], label=f'lambda8_test (Mean Error: {mean_error_gp_lambda8_combined:.2g})', color='red')
plt.plot(range(len(error_gp_lambda9_test[:subset_size])), error_gp_lambda9_test[:subset_size], label=f'lambda9_test (Mean Error: {mean_error_gp_lambda9_test:.2g})', color='brown')

plt.xlabel('Time Step')
plt.ylabel('Absolute Error')
plt.title('Absolute Error Against Time for "Test" Datasets')
plt.legend()
plt.grid(True)


# Calculate distance from each point in the trajectory to the closest point on the unit circle
def distance_to_unit_circle_trajectory(x_positions, y_positions):
    distances = []
    for x, y in zip(x_positions, y_positions):
        min_distance = float('inf')
        for theta in np.linspace(0, 2*np.pi, 100):
            unit_x = 1 + np.cos(theta)
            unit_y = np.sin(theta)
            distance = np.sqrt((x - unit_x)**2 + (y - unit_y)**2)
            if distance < min_distance:
                min_distance = distance
        distances.append(min_distance)
    return distances

# Calculate distances for each dataset
distances_gp_nogp_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp)
distances_gp_lambda01_combined_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda01_combined, actual_positions_y_gp_lambda01_combined)
distances_gp_lambda8_combined_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda8_combined, actual_positions_y_gp_lambda8_combined)
distances_gp_mac_test_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_mac_test, actual_positions_y_gp_mac_test)
distances_gp_lambda9_test_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda9_test, actual_positions_y_gp_lambda9_test)

# Calculate mean distances for each dataset
mean_distance_gp_nogp_closest = np.mean(distances_gp_nogp_closest)
mean_distance_gp_lambda01_combined_closest = np.mean(distances_gp_lambda01_combined_closest)
mean_distance_gp_lambda8_combined_closest = np.mean(distances_gp_lambda8_combined_closest)
mean_distance_gp_mac_test_closest = np.mean(distances_gp_mac_test_closest)
mean_distance_gp_lambda9_test_closest = np.mean(distances_gp_lambda9_test_closest)

# Plotting the distances
plt.figure(figsize=(12, 6))

plt.plot(range(len(distances_gp_nogp_closest[:subset_size])), distances_gp_nogp_closest[:subset_size], label=f'No GP (Mean Distance: {mean_distance_gp_nogp_closest:.2g})', color='blue')
plt.plot(range(len(distances_gp_lambda01_combined_closest[:subset_size])), distances_gp_lambda01_combined_closest[:subset_size], label=f'lambda = 1.0 (Mean Distance: {mean_distance_gp_lambda01_combined_closest:.2g})', color='green')
plt.plot(range(len(distances_gp_lambda8_combined_closest[:subset_size])), distances_gp_lambda8_combined_closest[:subset_size], label=f'lambda = 0.8 (Mean Distance: {mean_distance_gp_lambda8_combined_closest:.2g})', color='red')
plt.plot(range(len(distances_gp_mac_test_closest[:subset_size])), distances_gp_mac_test_closest[:subset_size], label=f'Ours (Mean Distance: {mean_distance_gp_mac_test_closest:.2g})', color='cyan')
plt.plot(range(len(distances_gp_lambda9_test_closest[:subset_size])), distances_gp_lambda9_test_closest[:subset_size], label=f'lambda = 0.9 (Mean Distance: {mean_distance_gp_lambda9_test_closest:.2g})', color='brown')

plt.xlabel('Time Step')
plt.ylabel('Distance to Closest Point on Unit Circle')
plt.title('Distance to Closest Point on Unit Circle Against Time')
plt.legend()
plt.grid(True)


# Calculate distances for "sine" datasets
distances_gp_nogp_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp)
distances_gp_lambda9_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine)
distances_gp_mac_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine)

# Calculate mean distances for "sine" datasets
mean_distance_gp_nogp_sine_closest = np.mean(distances_gp_nogp_sine_closest)
mean_distance_gp_lambda9_sine_closest = np.mean(distances_gp_lambda9_sine_closest)
mean_distance_gp_mac_sine_closest = np.mean(distances_gp_mac_sine_closest)

# Plotting the distances for "sine" datasets as well
plt.figure(figsize=(12, 6))

plt.plot(range(len(distances_gp_nogp_closest[:subset_size])), distances_gp_nogp_sine_closest[:subset_size], label=f'No GP (Mean Distance: {mean_distance_gp_nogp_sine_closest:.2g})', color='blue')
plt.plot(range(len(distances_gp_lambda9_sine_closest[:subset_size])), distances_gp_lambda9_sine_closest[:subset_size], label=f'lambda = 0.9 (Mean Distance: {mean_distance_gp_lambda9_sine_closest:.2g})', color='orange')
plt.plot(range(len(distances_gp_mac_sine_closest[:subset_size])), distances_gp_mac_sine_closest[:subset_size], label=f'Ours (Mean Distance: {mean_distance_gp_mac_sine_closest:.2g})', color='cyan')

plt.xlabel('Time Step')
plt.ylabel('Distance to Closest Point on Unit Circle')
plt.title('Distance to Closest Point on Unit Circle Against Time (Sine Datasets)')
plt.legend()
plt.grid(True)

# Plotting Trajectories
plt.figure(figsize=(12, 6))

# Plot for "sine" datasets
plt.subplot(1, 2, 1)
plt.plot(actual_positions_x_gp_nogp[:subset_size], actual_positions_y_gp_nogp[:subset_size], linewidth=3, label=f'No GP (Mean Error: {mean_distance_gp_nogp_closest:.2f})', color='blue')

plt.plot(actual_positions_x_gp_lambda01[:subset_size], actual_positions_y_gp_lambda01[:subset_size], linewidth=3, label=f'lambda = 1.0 (Mean Error: {mean_error_gp_lambda01:.2f})', color='green')
plt.plot(actual_positions_x_gp_lambda9_sine[:subset_size], actual_positions_y_gp_lambda9_sine[:subset_size], linewidth=2, label=f'lambda = 0.9 (Mean Error: {mean_error_gp_lambda9_sine:.2f})', color='orange')

plt.plot(actual_positions_x_gp_lambda8[:subset_size], actual_positions_y_gp_lambda8[:subset_size], linewidth=2, label=f'lambda = 0.8 (Mean Error: {mean_error_gp_lambda8:.2f})', color='purple')
plt.plot(actual_positions_x_gp_mac_sine[:subset_size], actual_positions_y_gp_mac_sine[:subset_size], linewidth=2, label=f'Ours (Mean Error: {mean_error_gp_mac_sine:.2f})', color='cyan')
# Plot for the unit circle (Reference)
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(1 + np.cos(theta), np.sin(theta), linestyle='--', linewidth=4, label='Reference', color='black')
plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Trajectories for "Sine Wave" Disturbance')
plt.legend()
plt.grid(True)

# Plot for "test" datasets
plt.subplot(1, 2, 2)
plt.plot(actual_positions_x_gp_nogp_test[:subset_size], actual_positions_y_gp_nogp_test[:subset_size], linewidth=3, label=f'No GP (Mean Error: {mean_distance_gp_nogp_closest:.2f})', color='blue')
plt.plot(actual_positions_x_gp_lambda01_combined[:subset_size], actual_positions_y_gp_lambda01_combined[:subset_size], linewidth=2, label=f'lambda = 1.0  (Mean Error: {mean_distance_gp_lambda01_combined_closest:.2f})', color='green')
#plt.plot(actual_positions_x_gp_lambda9_test[:subset_size], actual_positions_y_gp_lambda9_test[:subset_size], linewidth=2, label=f'lambda = 0.9 (Mean Error: {mean_error_gp_lambda9_test:.2f})', color='orange')
plt.plot(actual_positions_x_gp_lambda8_combined[:subset_size], actual_positions_y_gp_lambda8_combined[:subset_size], linewidth=2, label=f'lambda = 0.8  (Mean Error: {mean_distance_gp_lambda8_combined_closest:.2f})', color='purple')
plt.plot(actual_positions_x_gp_mac_test[:subset_size], actual_positions_y_gp_mac_test[:subset_size], linewidth=2, label=f'Ours (Mean Error: {mean_distance_gp_mac_test_closest:.2f})', color='cyan')


# Plot for the unit circle (Reference)
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(1 + np.cos(theta), np.sin(theta), linestyle='--', linewidth=4, label='Reference', color='black')
plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Trajectories for "Step" Disturbance')
plt.legend()
plt.grid(True)

plt.figure(figsize=(10, 8))

# Plot W1 for each text file
#plt.subplot(3, 2, 1)
#plt.plot(W1[:len(actual_positions_x_gp_mac_test)], label="GP_nogp")
plt.plot(mu_y_gp_mac_test, label="mu_y")

plt.plot(gt_y_gp_mac_test, label="gt_y")


plt.legend()
plt.title('W1')



plt.show()
