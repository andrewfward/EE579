import csv
import matplotlib.pyplot as plt

# === PARAMETERS ===
TOLERANCE = 2       # cm
MIN_SEQUENCE = 10   # points
FILENAME = "Bluetooth/FrontUltrasoundTests/MaxMotion/find110.csv" 

# === Load and parse data ===
data = []
with open(FILENAME, 'r') as f:
    for line in f:
        if ',' in line:
            try:
                angle, dist = map(float, line.strip().split(','))
                data.append((angle, dist))
            except ValueError:
                continue  # Skip headers or bad lines

angles = [d[0] for d in data]
distances = [d[1] for d in data]

# === Find candidate flat regions ===
sequences = []
start = 0
while start < len(data) - MIN_SEQUENCE:
    ref_dist = data[start][1]
    seq = [data[start]]

    for i in range(start + 1, len(data)):
        if abs(data[i][1] - ref_dist) <= TOLERANCE:
            seq.append(data[i])
        else:
            break

    if len(seq) >= MIN_SEQUENCE:
        before_idx = start - 1
        after_idx = start + len(seq)

        before_ok = before_idx >= 0 and data[before_idx][1] > ref_dist + TOLERANCE
        after_ok = after_idx < len(data) and data[after_idx][1] > ref_dist + TOLERANCE

        if before_ok or after_ok:
            mid = seq[len(seq) // 2]
            sequences.append({
                'start_angle': seq[0][0],
                'end_angle': seq[-1][0],
                'mid_angle': mid[0],
                'min_distance': min(x[1] for x in seq),
                'length': len(seq),
                'points': seq
            })
            start += len(seq)
        else:
            start += 1
    else:
        start += 1

# === Output ===
if sequences:
    print("Detected can candidates:")
    for seq in sequences:
        print(f"- Angle {seq['mid_angle']}°, range: [{seq['start_angle']}°–{seq['end_angle']}°], "
              f"min distance: {seq['min_distance']}cm, length: {seq['length']}")
else:
    print("No can-like regions found.")

# === Plotting ===
plt.figure(figsize=(12, 6))
plt.plot(angles, distances, label="Ultrasound Distance", color='blue')

# Highlight detected flat regions
for seq in sequences:
    seq_angles = [p[0] for p in seq['points']]
    seq_distances = [p[1] for p in seq['points']]
    plt.plot(seq_angles, seq_distances, color='red', linewidth=3, label="Can Region" if seq == sequences[0] else "")

    # Mid-angle marker
    plt.axvline(x=seq['mid_angle'], color='green', linestyle='--', alpha=0.7)

plt.xlabel("Angle (Us)")
plt.ylabel("Distance (cm)")
plt.title("Ultrasound Scan with Detected Can Regions")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
