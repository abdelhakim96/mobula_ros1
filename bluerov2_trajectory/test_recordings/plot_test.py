import os
import matplotlib.pyplot as plt
import numpy as np

# Define the folder name where the files are located
folder_name = ""


#7
# Define file paths for the recorded data
file_names = [
    "nrec3.txt",
    "nrec2.txt",
    "rec8.txt",
    "rec1.txt",
   # "real_gp5_t3.txt"
]

# Define plot duration (number of points to plot)
plot_duration = 600
# Define the starting point of plotting
start_point = 300 # Adjust as needed

# Lists to store trajectory data for each file
trajectory_data = {}

# Function to read data from a file and populate the lists
def read_data(file_path, data_dict):
    try:
        with open(file_path, 'r') as file:
            for line in file:
                data = line.strip().split(',')
                data_dict["ref_x"].append(float(data[0]))
                data_dict["ref_y"].append(float(data[1]))
                data_dict["actual_x"].append(float(data[3]))
                data_dict["actual_y"].append(float(data[4]))
    except Exception as e:
        print(f"Error reading data from {file_path}: {e}")

# Read data from each test file
for idx, file_name in enumerate(file_names, start=1):
    trajectory_data[f"test{idx}"] = {"ref_x": [], "ref_y": [], "actual_x": [], "actual_y": []}
    read_data(os.path.join(folder_name, file_name), trajectory_data[f"test{idx}"])

# Calculate error for each file
errors = {}
for test, data in trajectory_data.items():
    errors[test] = [np.sqrt((actual_x - ref_x)**2 + (actual_y - ref_y)**2) for actual_x, actual_y, ref_x, ref_y in zip(data["actual_x"], data["actual_y"], data["ref_x"], data["ref_y"])]

# Calculate mean error for each file
mean_errors = {test: np.mean(error_list) for test, error_list in errors.items()}

# Plot trajectories for each file
plt.figure(figsize=(10, 6))

# Plot reference trajectories
for test, color, label in zip(trajectory_data.keys(), ['blue', 'red', 'green'], ['Lambda = 0.8', 'DFGP', 'NOGP']):
    plt.plot(trajectory_data[test]["ref_x"][start_point:plot_duration], trajectory_data[test]["ref_y"][start_point:plot_duration], linestyle='--', label=f'{label} (Reference)', color='black')

# Plot actual trajectories (from start_point to plot_duration)
for test, color, label in zip(trajectory_data.keys(), ['blue', 'red', 'green'], ['Lambda = 0.8', 'DFGP', 'NOGP']):
    plt.plot(trajectory_data[test]["actual_x"][start_point:plot_duration], trajectory_data[test]["actual_y"][start_point:plot_duration], label=label, color=color)

plt.xlabel('X')
plt.ylabel('Y')
plt.title('2-D Trajectories')
plt.legend(loc='upper left')
plt.grid(True)
plt.show()

# Plot errors for each file
plt.figure(figsize=(10, 6))

for test, color, label in zip(errors.keys(), ['blue', 'red', 'green'], ['Lambda = 0.8', 'DFGP', 'NOGP']):
    plt.plot(errors[test][start_point:plot_duration], label=f'{label} Mean Error: {mean_errors[test]:.4f}', color=color)

plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error from Reference')
plt.legend(loc='upper left')
plt.grid(True)
plt.show()
