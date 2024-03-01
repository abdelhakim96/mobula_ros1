import os
import matplotlib.pyplot as plt

plt.close('all')

folder_name = ""

# Corrected file path containing the recorded data
file_name_testdata = os.path.join(folder_name, "testdata.txt")

# Lists to store the trajectory data
positions_x_testdata = []
positions_y_testdata = []

# Function to read data from a file and populate the lists
def read_data(file_path, x_list, y_list):
    try:
        with open(file_path, 'r') as file:
            for line in file:
                data = line.strip().split(',')
                x_list.append(float(data[3]) - 6376978)
                y_list.append(float(data[4]) - 1673721)
    except Exception as e:
        print(f"Error reading data from {file_path}: {e}")

# Read data from the "testdata.txt" file
read_data(file_name_testdata, positions_x_testdata, positions_y_testdata)

# Neglect the first 10 data points
positions_x_testdata = positions_x_testdata[10:]
positions_y_testdata = positions_y_testdata[10:]

# Plotting the positions against time
plt.figure(figsize=(15, 10))

# Plot x position against time
plt.subplot(2, 1, 1)
plt.plot(range(10, len(positions_x_testdata) + 10), positions_x_testdata, color='blue')
plt.xlabel('Time')
plt.ylabel('X Position')

# Plot y position against time
plt.subplot(2, 1, 2)
plt.plot(range(10, len(positions_y_testdata) + 10), positions_y_testdata, color='green')
plt.xlabel('Time')
plt.ylabel('Y Position')

plt.tight_layout()
plt.show()
