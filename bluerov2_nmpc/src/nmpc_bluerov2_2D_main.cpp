import os
import matplotlib.pyplot as plt
import numpy as np

# Define the folder name where the files are located
folder_name = ""

# Define file paths for the recorded data
file_names = [
    "real_gp8_t3.txt",
    "real_dfgp_t4.txt",
    "real_nogp_t4.txt",
    "real_gp1_t3.txt",
    "real_gp5_t3.txt"
]

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

# Plot trajectories and errors for each file
plt.figure(figsize=(10, 6))

# Plot reference trajectories
for idx, test in enumerate(trajectory_data.keys(), start=1):
    plt.plot(trajectory_data[test]["ref_x"][:300], trajectory_data[test]["ref_y"][:300], label=f'Reference Trajectory (File {idx})', linestyle='--')

# Plot actual trajectories (first 300 points only)
for idx, test in enumerate(trajectory_data.keys(), start=1):
    plt.plot(trajectory_data[test]["actual_x"][:300], trajectory_data[test]["actual_y"][:300], label=f'File {idx}', color=np.random.rand(3,))

plt.xlabel('X')
plt.ylabel('Y')
plt.title('2-D Trajectories')

# Add mean error to legend
for test, mean_error in mean_errors.items():
    plt.text(trajectory_data[test]["actual_x"][:10], trajectory_data[test]["actual_y"][:10], f"Mean Error: {mean_error:.4f}", color='black')

plt.legend(loc='upper left')
plt.grid(True)
plt.show()

# Plot errors for each file
plt.figure(figsize=(10, 6))

for idx, (test, error_list) in enumerate(errors.items(), start=1):
    plt.plot(error_list[:300], label=f'File {idx} Mean Error: {mean_errors[test]:.4f}')

plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error from Reference')
plt.legend(loc='upper left')
plt.grid(True)
plt.show()
