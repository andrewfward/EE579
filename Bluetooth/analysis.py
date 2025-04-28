import csv
import matplotlib.pyplot as plt

# === CONFIG ===
FILENAME = "Bluetooth/test9.csv"  # <- CHANGE this to your actual CSV filename
TOLERANCE = 0.1  # 10% tolerance for movement classification

# === Load CSV Data ===
distance_left = []
distance_right = []
steering_angle = []
pos = []  # To hold the 'Pos' data

with open(FILENAME, 'r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        distance_left.append(float(row['DistanceLeft']))
        distance_right.append(float(row['DistanceRight']))
        steering_angle.append(float(row['SteeringAngle']))
        pos.append(float(row['Pos']))  # Read the 'Pos' value

# === Calculate Differences ===
diffs = []
movement_classification = []

for i in range(1, len(distance_left)):
    delta_left = distance_left[i] - distance_left[i-1]
    delta_right = distance_right[i] - distance_right[i-1]

    diff = distance_left[i] - distance_right[i]
    diffs.append(diff)

    # Movement classification
    if abs(delta_left + delta_right) < TOLERANCE * max(abs(delta_left), abs(delta_right), 1):
        movement_classification.append("Drifting")
    elif abs(delta_left - delta_right) < TOLERANCE * max(abs(delta_left), abs(delta_right), 1):
        movement_classification.append("Walls Moving")
    else:
        movement_classification.append("Uncertain")

# === Plotting ===
time = list(range(len(distance_left)))

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12), sharex=True)

# Plot distances
ax1.plot(time, distance_left, label='Distance Left (cm)', color='blue')
ax1.plot(time, distance_right, label='Distance Right (cm)', color='red')
ax1.set_ylabel('Distance (cm)')
ax1.set_title('Left and Right Distance Over Time')
ax1.legend()
ax1.grid(True)

# Plot difference
ax2.plot(time[1:], diffs, label='Left - Right Difference (cm)', color='green')
ax2.set_xlabel('Time Step')
ax2.set_ylabel('Distance Difference (cm)')
ax2.set_title('Distance Difference Over Time (Drift vs Wall Movement)')
ax2.grid(True)

# Mark regions by movement type
for i, move_type in enumerate(movement_classification):
    if move_type == "Drifting":
        ax2.axvspan(i, i+1, color='orange', alpha=0.2, label='Drifting' if i == 0 else "")
    elif move_type == "Walls Moving":
        ax2.axvspan(i, i+1, color='cyan', alpha=0.2, label='Walls Moving' if i == 0 else "")

# Avoid duplicate legends
handles, labels = ax2.get_legend_handles_labels()
by_label = dict(zip(labels, handles))
ax2.legend(by_label.values(), by_label.keys())

# Plot Pos value
ax3.plot(time, pos, label='Position (Pos)', color='purple')
ax3.set_xlabel('Time Step')
ax3.set_ylabel('Position')
ax3.set_title('Position Over Time')
ax3.legend()
ax3.grid(True)

plt.tight_layout()
plt.show()
