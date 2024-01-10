import matplotlib.pyplot as plt

# File path containing the recorded data
file_name = "recorded_data.txt"

# Lists to store the trajectory data
ref_positions_x = []
ref_positions_y = []
actual_positions_x = []
actual_positions_y = []

# Read data from the file
with open(file_name, 'r') as file:
    for line in file:
        data = line.strip().split(',')
        ref_positions_x.append(float(data[0]))
        ref_positions_y.append(float(data[1]))
        actual_positions_x.append(float(data[4]))  # Assuming actual X position is in the 5th position
        actual_positions_y.append(float(data[5]))  # Assuming actual Y position is in the 6th position

# Plotting the trajectories
plt.figure(figsize=(8, 6))
plt.plot(ref_positions_x, ref_positions_y, label='Reference Trajectory', color='red')
plt.plot(actual_positions_x, actual_positions_y, label='Actual Trajectory', color='blue')
plt.xlabel('X-Position')
plt.ylabel('Y-Position')
plt.title('Reference vs Actual Trajectory')
plt.legend()
plt.grid(True)
plt.show()
