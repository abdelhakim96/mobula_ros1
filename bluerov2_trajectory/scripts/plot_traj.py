import matplotlib.pyplot as plt

# File paths containing the recorded data
file_name_gp = "recorded_data_gp.txt"
file_name_nogp2 = "recorded_data_nogp2.txt"

# Lists to store the trajectory data
ref_positions_x_gp = []
ref_positions_y_gp = []
actual_positions_x_gp = []
actual_positions_y_gp = []

ref_positions_x_nogp2 = []
ref_positions_y_nogp2 = []
actual_positions_x_nogp2 = []
actual_positions_y_nogp2 = []

# Read data from the first file
with open(file_name_gp, 'r') as file_gp:
    for line in file_gp:
        data = line.strip().split(',')
        ref_positions_x_gp.append(float(data[0]))
        ref_positions_y_gp.append(float(data[1]))
        actual_positions_x_gp.append(float(data[3]))  # Assuming actual X position is in the 5th position
        actual_positions_y_gp.append(float(data[4]))  # Assuming actual Y position is in the 6th position

# Read data from the second file
with open(file_name_nogp2, 'r') as file_nogp2:
    for line in file_nogp2:
        data = line.strip().split(',')
        ref_positions_x_nogp2.append(float(data[0]))
        ref_positions_y_nogp2.append(float(data[1]))
        actual_positions_x_nogp2.append(float(data[3]))  # Assuming actual X position is in the 5th position
        actual_positions_y_nogp2.append(float(data[4]))  # Assuming actual Y position is in the 6th position

# Plotting the trajectories
plt.figure(figsize=(12, 6))

# Plot for the first trajectory (recorded_data_gp.txt)
plt.subplot(1, 2, 1)
plt.plot(ref_positions_x_gp, ref_positions_y_gp, label='Reference Trajectory (GP)', color='red')
plt.plot(actual_positions_x_gp, actual_positions_y_gp, label='Actual Trajectory (GP)', color='blue')
plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Reference vs Actual Trajectory (GP)')
plt.legend()
plt.grid(True)

# Plot for the second trajectory (recorded_data_nogp2.txt)
plt.subplot(1, 2, 2)
plt.plot(ref_positions_x_nogp2, ref_positions_y_nogp2, label='Reference Trajectory (No GP)', color='red')
plt.plot(actual_positions_x_nogp2, actual_positions_y_nogp2, label='Actual Trajectory (No GP)', color='blue')
plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Reference vs Actual Trajectory (No GP)')
plt.legend()
plt.grid(True)

plt.tight_layout()  # Adjust layout for better appearance
plt.show()
