import tkinter as tk
from tkinter import filedialog
import serial
import threading
import csv
import time

class RoverControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Rover Control")

        self.serial_port = None
        self.running = False
        self.data_log = []

        self.connect_frame = tk.Frame(root)
        self.connect_frame.pack(pady=10)

        tk.Label(self.connect_frame, text="Bluetooth COM Port:").grid(row=0, column=0)
        self.port_entry = tk.Entry(self.connect_frame)
        self.port_entry.grid(row=0, column=1)
        self.connect_button = tk.Button(self.connect_frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=2)

        self.control_frame = tk.Frame(root)
        self.control_frame.pack(pady=10)

        self.start_button = tk.Button(self.control_frame, text="Start", command=self.send_start, state='disabled')
        self.start_button.grid(row=0, column=0, padx=5)
        self.stop_button = tk.Button(self.control_frame, text="Stop", command=self.send_stop, state='disabled')
        self.stop_button.grid(row=0, column=1, padx=5)
        self.save_button = tk.Button(self.control_frame, text="Save Log", command=self.save_log, state='disabled')
        self.save_button.grid(row=0, column=2, padx=5)

        self.log_frame = tk.Frame(root)
        self.log_frame.pack()

        self.text_log = tk.Text(self.log_frame, height=20, width=60)
        self.text_log.pack()

    def connect(self):
        port = self.port_entry.get()
        try:
            self.serial_port = serial.Serial(port, 9600, timeout=1)
            self.start_button.config(state='normal')
            self.stop_button.config(state='normal')
            self.save_button.config(state='normal')
            self.running = True
            threading.Thread(target=self.read_serial, daemon=True).start()

            # Send ping after connection
            self.send_ping()

        except Exception as e:
            print(f"Error: {e}")

    def send_start(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'start\n')

    def send_stop(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'stop\n')

    def send_ping(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'ping\n')

    def read_serial(self):
        while self.running:
            if self.serial_port.in_waiting:
                line = self.serial_port.readline().decode('utf-8').strip()
                if line:
                    if line == "Pong":
                        self.text_log.insert(tk.END, "Connection verified: Pong received from ESP32\n")
                    elif line == "Start Command Received":
                        self.text_log.insert(tk.END, "Start command confirmed.\n")
                        # Change button colors without pop-up
                        self.start_button.config(bg="green", fg="white")  # Change Start button color
                        self.stop_button.config(bg="red", fg="white")    # Change Stop button color
                    elif line == "Stop Command Received":
                        self.text_log.insert(tk.END, "Stop command confirmed.\n")
                        # Change button colors without pop-up
                        self.stop_button.config(bg="green", fg="white")  # Change Stop button color
                        self.start_button.config(bg="red", fg="white")   # Change Start button color
                    elif line.startswith("CAN:"):
                        self.text_log.insert(tk.END, f"*** {line[4:].strip()} ***\n")  # remove the 'CAN:' prefix
                        self.text_log.see(tk.END)
                    else:
                        # Assuming data comes in the format: DistanceLeft,DistanceRight,SteeringAngle,Pos
                        data_parts = line.split(',')
                        if len(data_parts) == 4:  # Make sure there are 4 parts
                            distance_left = data_parts[0]
                            distance_right = data_parts[1]
                            steering_angle = data_parts[2]
                            pos = data_parts[3]
                            # Append the data to the log with the new "pos" column
                            self.data_log.append([distance_left, distance_right, steering_angle, pos])
                            self.text_log.insert(tk.END, f"{line}\n")
                        if len(data_parts) == 2:
                            distance_left = data_parts[0]
                            distance_right = data_parts[1]
                            steering_angle = data_parts[0]
                            pos = data_parts[1]
                            self.data_log.append([distance_left, distance_right, steering_angle, pos])
                            self.text_log.insert(tk.END, f"{line}\n")
                    self.text_log.see(tk.END)
            time.sleep(0.05)

    def save_log(self):
        if self.data_log:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                    filetypes=[("CSV files", "*.csv")])
            if filepath:
                with open(filepath, 'w', newline='') as f:
                    writer = csv.writer(f)
                    # Add the new header with the "Pos" column
                    writer.writerow(["DistanceLeft", "DistanceRight", "SteeringAngle", "Pos"])
                    writer.writerows(self.data_log)

    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = RoverControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
