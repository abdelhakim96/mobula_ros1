import os
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

plt.close('all')

folder_name = ""

# Corrected file paths containing the recorded data

file_name_gp_nogp = os.path.join(folder_name, "real_nogp1.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "rec13.txt")
file_name_gp_lambda01_combined = os.path.join(folder_name, "rec8.txt")
file_name_gp_lambda8 = os.path.join(folder_name, "rec1.txt")
file_name_gp_lambda8_combined = os.path.join(folder_name, "rec1.txt")
file_name_gp_mac_sine = os.path.join(folder_name, "23.txt")
file_name_gp_mac_test = os.path.join(folder_name, "23.txt")
file_name_gp_lambda9_sine = os.path.join(folder_name, "rec8.txt")
file_name_gp_lambda9_test = os.path.join(folder_name, "rec8.txt")
file_name_nogp_test = os.path.join(folder_name, "real_nogp1.txt")

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
W3 = []

W1_mac = []
W2_mac = []
W3_mac = []
mu_y = []
gt_y = []
gt_y_1 = []

mu_y_1 = []
gt_y_1 = []
mu_y_9 = []
gt_y_9 = []
mu_y_8 = []
mu_y_mac = []
gt_y_8 = []

gt_y_nogp = []
gt_y_mac = []

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
                W1_list.append(float(data[20]))
                W2_list.append(float(data[21]))
                W3_list.append(float(data[22]))
                mu_y_list.append(float(data[27]))
                gt_y_list.append(float(data[28]))

    except Exception as e:
        print(f"Error reading data from {file_path}: {e}")

# Read data from files
read_data(file_name_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, W1, W2, W3, mu_y, gt_y_nogp)
read_data(file_name_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, W1, W2, W3, mu_y_1, gt_y_1)
read_data(file_name_gp_lambda8, ref_positions_x_gp_lambda8, ref_positions_y_gp_lambda8, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, W1, W2, W3, mu_y_8, gt_y_8)
read_data(file_name_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, W1_mac, W2_mac, W3_mac, mu_y_mac,  gt_y_mac)
read_data(file_name_gp_lambda9_sine, ref_positions_x_gp_lambda9_sine, ref_positions_y_gp_lambda9_sine, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, W1, W2, W3, mu_y_9, gt_y_9)

# Start time and duration parameters
t_start = 120
duration = 700
subset_size = duration

colors = ['#a1cb58', '#e7c550', '#1252cf', '#6E091A', '#F7A400']

# Calculate distance from each point in the trajectory to the closest point on the reference trajectory of the DF-GP method
def distance_to_reference_trajectory(x_positions, y_positions, ref_x_positions, ref_y_positions):
    distances = []
    for x, y in zip(x_positions, y_positions):
        min_distance = float('inf')
        for ref_x, ref_y in zip(ref_x_positions, ref_y_positions):
            distance = np.sqrt((x - ref_x)**2 + (y - ref_y)**2)
            if distance < min_distance:
                min_distance = distance
        distances.append(min_distance)
    return distances

# Calculate distances for the DF-GP method
distances_gp_mac_sine_closest_to_ref = distance_to_reference_trajectory(actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine)
distances_gp_nogp_closest_to_ref = distance_to_reference_trajectory(actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine)



# Calculate mean distances for the DF-GP method
mean_distance_gp_mac_sine_closest_to_ref = np.mean(distances_gp_mac_sine_closest_to_ref[:duration])
mean_distance_nogp_closest_to_ref = np.mean(distances_gp_nogp_closest_to_ref [:duration])



pdf_file_path = "plots.pdf"
pdf_pages = PdfPages(pdf_file_path)

# Plotting the distances
plt.figure(figsize=(15, 10))

dur = duration
t_end = dur   # Calculate end time

# Plot the distances for the DF-GP method using the reference trajectory
skip = 5  # Skip every 5 points
plt.plot(np.array(range(t_start, t_end, skip))/10, distances_gp_nogp_closest_to_ref[t_start:t_end:skip], linewidth=4, color=colors[4], label=f'No-GP (Mean: {mean_distance_nogp_closest_to_ref:.3g})')

plt.plot(np.array(range(t_start, t_end, skip))/10, distances_gp_mac_sine_closest_to_ref[t_start:t_end:skip], linewidth=4, color=colors[3], label=f'DF-GP (Mean: {mean_distance_gp_mac_sine_closest_to_ref:.3g})')
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.xlabel('X [m]', fontsize=20)
plt.ylabel('Y [m]', fontsize=20)

# Add labels and title
plt.xlabel('Time', fontsize=30)
plt.ylabel('Absolute Error [m]', fontsize=30)
#plt.title('Error in Trajectory Tracking', fontsize=30)


# Add legend
plt.legend(fontsize=30)

plt.grid(True)
pdf_pages.savefig()

# Plotting Trajectories

# Function to plot trajectories for a subset of data
def plot_trajectories(ax, actual_x, actual_y, label, t_start, duration, color):
    ax.plot(actual_x[t_start:t_start + duration], actual_y[t_start:t_start + duration], linewidth=4, label=label, color=color)

# Function to plot gt_y for each method
def plot_dist(ax, gt_y_values, label, t_start, duration, color):
    ax.plot(np.array(range(t_start, t_start + duration))/2, gt_y_values[t_start:t_start + duration], label=label, color=color, linewidth=4)

def plot_dist1(ax, gt_y_values, label, t_start, duration, color):
    ax.plot(np.array(range(t_start, t_start + duration))/10, gt_y_values[t_start:t_start + duration], label=label, color=color, linestyle='--', linewidth=4)

t1 = 100
t2 = 250
t3 = 400

plt.figure(figsize=(15, 10))

#plt.subplot(1, 2, 1)  # Subplot 1: First Cycle
theta = np.linspace(0, 2*np.pi, 100)
plot_trajectories(plt, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine, 'reference', t1, t2, color='black')
plot_trajectories(plt, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, 'No GP', t1, t2, color=colors[4])
#plot_trajectories(plt, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, 'Lambda = 1.0', t1, t2, color=colors[0])
#plot_trajectories(plt, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, 'Lambda = 0.8', t1, t2, color=colors[1])
#plot_trajectories(plt, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, 'Lambda = 0.5', t1, t2, color=colors[2])
plot_trajectories(plt, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, 'DF-GP', t1, t2, color=colors[3])
plt.legend(fontsize=20)
plt.xlabel('X [m]', fontsize=30)
plt.ylabel('Y [m]', fontsize=30)
#plt.title('First Cycle', fontsize=30)
plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio
plt.xticks(fontsize=25)
plt.yticks(fontsize=25)

t2 = t1 + t2
t3 = 320

pdf_pages.savefig()
plt.figure(figsize=(14, 10))


#plt.subplot(1, 2, 2)  # Subplot 2: Second Cycle
plot_trajectories(plt, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine, 'reference', t1, t3, color='black')
plot_trajectories(plt, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, 'No GP', t2, t3, color=colors[4])
#plot_trajectories(plt, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, 'Lambda = 1.0', t2, t3, color=colors[0])
#plot_trajectories(plt, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, 'Lambda = 0.8', t2, t3, color=colors[1])
#plot_trajectories(plt, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, 'Lambda = 0.5', t2, t3, color=colors[2])
plot_trajectories(plt, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, 'DF-GP', t2, t3, color=colors[3])
plt.legend(fontsize=30)
plt.xlabel('X [m]', fontsize=30)
plt.ylabel('Y [m]', fontsize=30)
#plt.title('Second Cycle', fontsize=30)
plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio
plt.xticks(fontsize=20)
plt.yticks(fontsize=25)

plt.tight_layout()  # Adjust layout to prevent overlap

pdf_pages.savefig()

plt.figure(figsize=(15, 10))

# Define t_start and duration
t_start = 1
duration = 700

# Plot gt_y for each method with skipping every 5 points and increased line thickness
#plot_dist(plt, mu_y_1[::5], 'lambda = 1', t_start*5, duration // 5, color=colors[0])
#plot_dist(plt, mu_y_8[::5], 'lambda = 0.8', t_start*5, duration // 5, color=colors[1])
#plot_dist(plt, mu_y_9[::5], 'lambda = 0.5', t_start*5, duration // 5, color=colors[2])
plot_dist(plt, mu_y_mac[::5], ' DF-GP ', t_start*5, duration // 5, color=colors[3])


gt_y_1_scaled = [y / 11.4 for y in gt_y_mac]
scaled_t_start = t_start * 5

# Call plot_dist1 with the scaled data and skipping every 5 points
plot_dist(plt, gt_y_mac[::5], 'ground truth', scaled_t_start, duration // 5, color='black')



plt.xlabel('Time [$\mathregular{s}$]', fontsize=30)
plt.ylabel('Disturbance [$\mathregular{m/s^2}$]', fontsize=30)
plt.legend(fontsize=30)
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)
plt.xlim(0, 70)

plt.grid(True)

pdf_pages.savefig()


def plot_dist1(ax, gt_y_values, mu_y_values, label, t_start, duration, color):
    abs_error = [abs(gt - mu) for gt, mu in zip(gt_y_values[t_start:t_start + duration], mu_y_values[t_start:t_start + duration])]
    mean_abs_error = np.mean(abs_error)
    ax.plot(np.array(range(t_start, t_start + duration))/10, abs_error, label=f'{label}', color=color, linewidth=4)

plt.figure(figsize=(15, 10))

# Define t_start and duration
t_start = 0
duration = 700

def plot_dist1(ax, gt_y_values, mu_y_values, label, t_start, duration, color):
    abs_error = [abs(gt - mu) for gt, mu in zip(gt_y_values[t_start:t_start + duration], mu_y_values[t_start:t_start + duration])]
    mean_abs_error = np.mean(abs_error)
    x_range = np.array(range(t_start, t_start + duration, 5))/10  # Adjusting the x-axis range
    ax.plot(x_range, abs_error[::5], label=f'{label}', color=color, linestyle='-', linewidth=4)

plt.figure(figsize=(15, 10))

# Define t_start and duration
t_start = 0
#duration = 820

# Plot error for different values of lambda
#plot_dist1(plt, gt_y_1_scaled, mu_y_1, 'Lambda = 1', t_start, duration, color=colors[0])
#plot_dist1(plt, gt_y_1_scaled, mu_y_8, 'Lambda = 0.8', t_start, duration, color=colors[1])
#plot_dist1(plt, gt_y_1_scaled, mu_y_9, 'Lambda = 0.5', t_start, duration, color=colors[2])
plot_dist1(plt, gt_y_mac, mu_y_mac, 'DF-GP', t_start, duration, color=colors[3])

# Add legend
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)

plt.legend(fontsize=30)
plt.xlabel('Time [s]', fontsize=30)
plt.ylabel('Absolute Error [$\mathregular{m/s^2}$]', fontsize=30)
plt.title('Error in Prediction', fontsize=30)
plt.grid(True)
pdf_pages.savefig()
plt.xlim(0, 70)


plt.figure(figsize=(15, 10))

# Plot the weights for the "macgp" method
skip = 1
plt.plot(np.array(range(t_start, t_end, skip))/10, W1_mac[t_start:t_end:skip], linewidth=4, label='W1', color=colors[0])
plt.plot(np.array(range(t_start, t_end, skip))/10, W2_mac[t_start:t_end:skip], linewidth=4, label='W2', color=colors[1])
plt.plot(np.array(range(t_start, t_end, skip))/10, W3_mac[t_start:t_end:skip], linewidth=4, label='W3', color=colors[2])

# Find which weight is the largest at each time instance
max_weights = np.argmax([W1_mac, W2_mac, W3_mac], axis=0)
colors = [colors[0], colors[1], colors[2]]
#labels = ['W1', 'W2', 'W3']

# Plot points on the x-axis based on the largest weight
for i in range(len(max_weights)):
    max_index = np.where(max_weights == i)[0]
    max_index = max_index[(max_index >= 0) & (max_index < len(W1_mac))]
    if len(max_index) > 0:
        plt.scatter(np.array([t_start + idx * skip for idx in max_index])/10, [0] * len(max_index), color=colors[i], zorder=5)

# Add labels and title
plt.xlabel('Time [s]', fontsize=30)
plt.ylabel('Weight Value', fontsize=30)
plt.title('Weights for DF-GP Method', fontsize=30)
plt.xticks(fontsize=30)
plt.yticks(fontsize=30)

# Add legend
plt.legend(fontsize=30)
pdf_pages.savefig()

pdf_pages.close()

print("PDF saved successfully.")
plt.show()
