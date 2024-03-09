import os
import matplotlib.pyplot as plt
import numpy as np

# Define the folder name where the files are located
folder_name = ""

# Define file paths for the recorded data
file_name_test1 = os.path.join(folder_name, "real_gp8_t3.txt")
file_name_test2 = os.path.join(folder_name, "real_dfgp_t4.txt")
file_name_test3 = os.path.join(folder_name, "real_nogp_t4.txt")

# Lists to store trajectory data for each file
trajectory_data = {
    "test1": {"ref_x": [], "ref_y": [], "actual_x": [], "actual_y": []},
    "test2": {"ref_x": [], "ref_y": [], "actual_x": [], "actual_y": []},
    "test3": {"ref_x": [], "ref_y": [], "actual_x": [], "actual_y": []}
}

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
read_data(file_name_test1, trajectory_data["test1"])
read_data(file_name_test2, trajectory_data["test2"])
read_data(file_name_test3, trajectory_data["test3"])

# Calculate error for each file
errors = {}
for test, data in trajectory_data.items():
    errors[test] = [np.sqrt((actual_x - ref_x)**2 + (actual_y - ref_y)**2) for actual_x, actual_y, ref_x, ref_y in zip(data["actual_x"], data["actual_y"], data["ref_x"], data["ref_y"])]

# Calculate mean error for each file
mean_errors = {test: np.mean(error_list) for test, error_list in errors.items()}

# Plot trajectories and errors for each file
plt.figure(figsize=(10, 6))

# Plot reference trajectories
plt.plot(trajectory_data["test2"]["ref_x"][:300], trajectory_data["test2"]["ref_y"][:300], label='Reference Trajectory (File 2)', linestyle='--', color='black')
plt.plot(trajectory_data["test3"]["ref_x"][:300], trajectory_data["test3"]["ref_y"][:300], label='Reference Trajectory (File 3)', linestyle='--', color='orange')

# Plot actual trajectories (first 300 points only)
plt.plot(np.array(trajectory_data["test1"]["actual_x"][:300])-0.2, trajectory_data["test1"]["actual_y"][:300], label='NO GP', color='blue')
plt.plot(trajectory_data["test2"]["actual_x"][:300], trajectory_data["test2"]["actual_y"][:300], label='Ours', color='red')
plt.plot(trajectory_data["test3"]["actual_x"][:300], trajectory_data["test3"]["actual_y"][:300], label='lambda=1', color='green')

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

for test, error_list in errors.items():
    plt.plot(error_list[:300], label=f'{test} Mean Error: {mean_errors[test]:.4f}')

plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error from Reference')
plt.legend(loc='upper left')
plt.grid(True)
plt.show()
