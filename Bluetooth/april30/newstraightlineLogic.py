import csv
import matplotlib.pyplot as plt

def process_sensor_data(csv_file_path, initialOffsetL, initialOffsetR):
    # Read data
    with open(csv_file_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # skip header
        data = [[int(row[0]), int(row[1])] for row in reader if row[0] and row[1]]

    pos_values = []

    lastL = data[0][0]
    lastR = data[0][1]

    for distanceL, distanceR in data:
        # Check for sudden change in left sensor
        if abs(distanceL - lastL) > 5:
            delta = distanceL - lastL
            initialOffsetL += delta
            print(f"Left offset updated by {delta} -> {initialOffsetL}")

        # Check for sudden change in right sensor
        if abs(distanceR - lastR) > 5:
            delta = distanceR - lastR
            initialOffsetR += delta
            print(f"Right offset updated by {delta} -> {initialOffsetR}")

        lastL = distanceL
        lastR = distanceR

        # Calculate corrected positions
        posL = distanceL - initialOffsetL
        posR = distanceR - initialOffsetR
        pos = posR - posL
        pos_values.append(pos)

    # Plotting the result
    plt.figure(figsize=(12, 6))
    plt.plot(pos_values, label="Corrected Pos (R - L)")
    plt.axhline(0, color='gray', linestyle='--')
    plt.title("Corrected Pos Over Time")
    plt.xlabel("Sample Index")
    plt.ylabel("Relative Position (cm)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

# ===== USER INPUTS HERE =====
csv_file_path = "Bluetooth/may6/test1.csv"  # Replace with your file path
initialOffsetL = int(input("Enter initial offset for LEFT sensor (cm): "))
initialOffsetR = int(input("Enter initial offset for RIGHT sensor (cm): "))

# Run the function
process_sensor_data(csv_file_path, initialOffsetL, initialOffsetR)

