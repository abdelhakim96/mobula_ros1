import os
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

plt.close('all')

folder_name = ""

# Corrected file paths containing the recorded data
file_name_gp_nogp = os.path.join(folder_name, "nogp.txt")
file_name_gp_lambda01 = os.path.join(folder_name, "1f.txt")
file_name_gp_lambda01_combined = os.path.join(folder_name, "1f.txt")
file_name_gp_lambda8 = os.path.join(folder_name, "0.8f.txt")
file_name_gp_lambda8_combined = os.path.join(folder_name, "0.8f.txt")
file_name_gp_mac_sine = os.path.join(folder_name, "macf.txt")
file_name_gp_mac_test = os.path.join(folder_name, "macf.txt")
file_name_gp_lambda9_sine = os.path.join(folder_name, "0.5f.txt")
file_name_gp_lambda9_test = os.path.join(folder_name, "0.5f.txt")
file_name_nogp_test = os.path.join(folder_name, "nogp.txt")

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
gt_y_1 =[] 


mu_y_1 =[]
gt_y_1 =[] 
mu_y_9 =[]
gt_y_9 =[] 
mu_y_8 =[]
mu_y_mac =[]
gt_y_8=[] 

gt_y_nogp=[] 
gt_y_mac =[]
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
                mu_y_list.append(float(data[27]))
                gt_y_list.append(float(data[24]))
                
    except Exception as e:
        print(f"Error reading data from {file_path}: {e}")






# Read data from files
read_data(file_name_gp_nogp, ref_positions_x_gp_nogp, ref_positions_y_gp_nogp, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, W1, W2, W3, mu_y, gt_y_nogp)
read_data(file_name_gp_lambda01, ref_positions_x_gp_lambda01, ref_positions_y_gp_lambda01, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, W1, W2, W3, mu_y_1, gt_y_1)
read_data(file_name_gp_lambda8, ref_positions_x_gp_lambda8, ref_positions_y_gp_lambda8, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, W1, W2, W3, mu_y_8, gt_y_8)
read_data(file_name_gp_mac_sine, ref_positions_x_gp_mac_sine, ref_positions_y_gp_mac_sine, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, W1, W2, W3, mu_y_mac,  gt_y_mac)
read_data(file_name_gp_lambda9_sine, ref_positions_x_gp_lambda9_sine, ref_positions_y_gp_lambda9_sine, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, W1, W2, W3, mu_y_9, gt_y_9)


# Start time and duration parameters
t_start = 1
duration = 620
subset_size= duration

colors = ['#9467bd','#2ca02c', '#d62728', '#1f77b4','#ff7f0e' ]

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
distances_gp_lambda1_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01)
distances_gp_lambda8_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8)
distances_gp_mac_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine)
distances_gp_lambda9_sine_closest = distance_to_unit_circle_trajectory(actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine)

# Calculate mean distances for each dataset
mean_distance_gp_nogp_closest = np.mean(distances_gp_nogp_closest[:duration])
mean_distance_gp_lambda01_sine_closest = np.mean(distances_gp_lambda1_sine_closest[:duration])
mean_distance_gp_lambda8_sine_closest = np.mean(distances_gp_lambda8_sine_closest[:duration])
mean_distance_gp_mac_sine_closest = np.mean(distances_gp_mac_sine_closest[:duration])
mean_distance_gp_lambda9_sine_closest = np.mean(distances_gp_lambda9_sine_closest[:duration])

pdf_file_path = "plots.pdf"
pdf_pages = PdfPages(pdf_file_path)



# Plotting the distances
plt.figure(figsize=(12, 6))

dur = duration 
t_end = t_start + dur   # Calculate end time

# Plot the distances for each dataset
skip = 10  # Skip every 10 points
plt.plot(range(t_start, t_end, skip), distances_gp_nogp_closest[t_start:t_end:skip], linewidth=2,color=colors[4], label=f'No GP (Mean: {mean_distance_gp_nogp_closest:.3g})')
plt.plot(range(t_start, t_end, skip), distances_gp_lambda1_sine_closest[t_start:t_end:skip], linewidth=2,color=colors[0], label=f'Lambda 1.0 (Mean: {mean_distance_gp_lambda01_sine_closest:.3g})')
plt.plot(range(t_start, t_end, skip), distances_gp_lambda8_sine_closest[t_start:t_end:skip],linewidth=2, color=colors[1], label=f'Lambda 0.8 (Mean: {mean_distance_gp_lambda8_sine_closest:.3g})')
plt.plot(range(t_start, t_end, skip), distances_gp_lambda9_sine_closest[t_start:t_end:skip],linewidth=2, color=colors[2], label=f'Lambda 0.5 (Mean: {mean_distance_gp_lambda9_sine_closest:.3g})')
plt.plot(range(t_start, t_end, skip), distances_gp_mac_sine_closest[t_start:t_end:skip], linewidth=2, color=colors[3], label=f'Ours (Mean: {mean_distance_gp_mac_sine_closest:.3g})')
# Add labels and title
plt.xlabel('Time')
plt.ylabel('Error')
plt.title('Error in Trajectory Tracking')

# Add legend
plt.legend()


plt.grid(True)
pdf_pages.savefig()
# Plotting Trajectories

# Function to plot trajectories for a subset of data
def plot_trajectories(ax, actual_x, actual_y, label, t_start, duration, color):
    ax.plot(actual_x[t_start:t_start + duration], actual_y[t_start:t_start + duration], linewidth=2, label=label, color=color)

# Function to plot gt_y for each method
def plot_dist(ax, gt_y_values, label, t_start, duration, color):
    ax.plot(range(t_start, t_start + duration), gt_y_values[t_start:t_start + duration], label=label, color=color)


def plot_dist1(ax, gt_y_values, label, t_start, duration, color):
    ax.plot(range(t_start, t_start + duration), gt_y_values[t_start:t_start + duration], label=label, color=color ,linestyle='--',linewidth=4 )

t1 = t_start
t2= 317
t3 = 630-317
#t2 = int(duration/2)

#t3 = int(duration-40)

plt.figure(figsize=(10, 10))

plt.subplot(1, 2, 1)  # Subplot 1: First Cycle
theta = np.linspace(0, 2*np.pi, 100)
plt.plot(1 + np.cos(theta), np.sin(theta), linestyle='--', linewidth=2, label='Reference', color='black')
plot_trajectories(plt, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, 'No GP', t_start, t2, color=colors[4])
plot_trajectories(plt, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, 'Lambda = 1.0', t_start, t2, color=colors[0])
plot_trajectories(plt, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, 'Lambda = 0.8', t_start, t2, color=colors[1])
plot_trajectories(plt, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, 'Lambda = 0.5', t_start, t2, color=colors[2])
plot_trajectories(plt, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, 'Ours', t_start, t2, color=colors[3])
plt.legend()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('First Cycle')
plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio


plt.subplot(1, 2, 2)  # Subplot 2: Second Cycle
plt.plot(1 + np.cos(theta), np.sin(theta), linestyle='--', linewidth=2, label='Reference', color='black')
plot_trajectories(plt, actual_positions_x_gp_nogp, actual_positions_y_gp_nogp, 'No GP', t2, t3, color=colors[4])
plot_trajectories(plt, actual_positions_x_gp_lambda01, actual_positions_y_gp_lambda01, 'Lambda = 1.0', t2, t3, color=colors[0])
plot_trajectories(plt, actual_positions_x_gp_lambda8, actual_positions_y_gp_lambda8, 'Lambda = 0.8', t2, t3, color=colors[1])
plot_trajectories(plt, actual_positions_x_gp_lambda9_sine, actual_positions_y_gp_lambda9_sine, 'Lambda = 0.5', t2, t3, color=colors[2])
plot_trajectories(plt, actual_positions_x_gp_mac_sine, actual_positions_y_gp_mac_sine, 'Ours', t2, t3, color=colors[3])
plt.legend()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Second Cycle')
plt.gca().set_aspect('equal', adjustable='box')  # Equal aspect ratio

plt.tight_layout()  # Adjust layout to prevent overlap



pdf_pages.savefig()
plt.figure(figsize=(12, 6))



# Plot gt_y for each method
#plot_gt_y(plt, gt_y_nogp, 'No GP', t_start, duration, 'blue')
plot_dist(plt, mu_y_1, 'lambda = 1', t_start, duration , color=colors[0])
plot_dist(plt, mu_y_8, 'lambda = 0.8', t_start, duration , color=colors[1])
plot_dist(plt, mu_y_9, 'lambda = 0.5', t_start, duration , color=colors[2])
plot_dist(plt, mu_y_mac, ' Ours ', t_start, duration , color=colors[3])
gt_y_1_scaled = [y / 11.4 for y in gt_y_1]

# Call plot_dist1 with the scaled data
plot_dist1(plt, gt_y_1_scaled, 'ground truth', t_start, duration, color='black')


plt.xlabel('Time')
plt.ylabel('Disturbance')
plt.title('Disturbance')
plt.legend()
plt.grid(True)

pdf_pages.savefig()

def plot_dist1(ax, gt_y_values, mu_y_values, label, t_start, duration, color):
    abs_error = [abs(gt - mu) for gt, mu in zip(gt_y_values[t_start:t_start + duration], mu_y_values[t_start:t_start + duration])]
    mean_abs_error = np.mean(abs_error)
    ax.plot(range(t_start, t_start + duration), abs_error, label=f'{label} (Mean Abs. Error: {mean_abs_error:.3g})', color=color, linestyle='--', linewidth=4)

plt.figure(figsize=(12, 6))

# Define t_start and duration
t_start = 0
#duration = 820

# Plot error for different values of lambda
plot_dist1(plt, gt_y_1_scaled, mu_y_1, 'Lambda = 1', t_start, duration, color=colors[0])
plot_dist1(plt, gt_y_1_scaled, mu_y_8, 'Lambda = 0.8', t_start, duration, color=colors[1])
plot_dist1(plt, gt_y_1_scaled, mu_y_9, 'Lambda = 0.5', t_start, duration, color=colors[2])
plot_dist1(plt, gt_y_1_scaled, mu_y_mac, 'Ours', t_start, duration, color=colors[3])

# Add legend
plt.legend()
# Show plot


plt.xlabel('Time')
plt.ylabel('Absolute Error ')
plt.title('Error in Prediction')
plt.legend()
plt.grid(True)


pdf_pages.savefig()
pdf_pages.close()

plt.show()



print("PDF saved successfully.")
