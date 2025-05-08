import tkinter as tk
from tkinter import ttk
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

        self.voltage_var = tk.DoubleVar(value=7.9)

        self.connect_frame = tk.Frame(root)
        self.connect_frame.pack(pady=5)

        tk.Label(self.connect_frame, text="Bluetooth COM Port:").grid(row=0, column=0)
        self.port_entry = tk.Entry(self.connect_frame)
        self.port_entry.grid(row=0, column=1)
        self.connect_button = tk.Button(self.connect_frame, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=2)

        self.voltage_frame = tk.Frame(root)
        self.voltage_frame.pack(pady=5)

        self.voltage_label = tk.Label(self.voltage_frame, text = 'Voltage')
        self.voltage_label.grid(row=0,column=0)
        self.voltage_entry = tk.Entry(self.voltage_frame, textvariable=self.voltage_var, state='disabled')
        self.voltage_entry.grid(row=0,column=1)
        self.submit_button = tk.Button(self.voltage_frame, text="Enter", command=self.submit_voltage, state='disabled')
        self.submit_button.grid(row=0,column=3)

        self.control_frame = tk.Frame(root)
        self.control_frame.pack(pady=10)

        self.start_button = tk.Button(self.control_frame, text="Start", command=self.send_start, state='disabled')
        self.start_button.grid(row=0, column=0, padx=5)
        self.stop_button = tk.Button(self.control_frame, text="Stop", command=self.send_stop, state='disabled')
        self.stop_button.grid(row=0, column=1, padx=5)
        self.save_button = tk.Button(self.control_frame, text="Save Log", command=self.save_log, state='disabled')
        self.save_button.grid(row=0, column=2, padx=5)
        
        self.offsets_frame = tk.Frame(root)
        self.offsets_frame.pack()
        
        self.sidevariable = tk.StringVar(root, '1')

        self.offsets_button = tk.Button(self.offsets_frame, text="Two-sided Offsets", command=self.calc_offsets, state='disabled')
        self.offsets_button.grid(row=0, column=0, padx=5, pady=5)
        self.onesided_offsets_button = tk.Button(self.offsets_frame, text="One-sided Offsets", command=self.unlock_radio_buttons, state='disabled')
        self.onesided_offsets_button.grid(row=0, column=1, columnspan=2, padx=5, pady=5)
        self.leftside_checkbox = tk.Button(self.offsets_frame, text="Left Side", command=self.select_side_left, state='disabled')
        self.leftside_checkbox.grid(row=1,column=1,padx=5, pady=5)
        self.rightside_checkbox = tk.Button(self.offsets_frame, text="Right Side", command=self.select_side_right, state='disabled')
        self.rightside_checkbox.grid(row=1,column=2,padx=5, pady=5)

        self.log_frame = tk.Frame(root)
        self.log_frame.pack(pady=10)

        self.text_log = tk.Text(self.log_frame, height=20, width=60)
        self.text_log.pack()

    def connect(self):
        port = self.port_entry.get()
        try:
            self.serial_port = serial.Serial(port, 9600, timeout=1)
            self.start_button.config(state='normal')
            self.stop_button.config(state='normal')
            self.voltage_entry.config(state='normal')
            self.save_button.config(state='normal')
            self.submit_button.config(state='normal')
            self.offsets_button.config(state='normal')
            self.onesided_offsets_button.config(state='normal')

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

    def calc_offsets(self):
        self.serial_port.write(b'calc_offsets\n')
        self.leftside_checkbox.config(state='disabled')
        self.rightside_checkbox.config(state='disabled')


    def on_closing(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.root.destroy()


    def unlock_radio_buttons(self):
        self.leftside_checkbox.config(state='normal')
        self.rightside_checkbox.config(state='normal')

    def select_side_left(self):
        if self.serial_port and self.serial_port.is_open:
            self.text_log.insert(tk.END, "Calculating offsets based on LHS\n")
            self.serial_port.write(b'LHS\n')

    def select_side_right(self):
        if self.serial_port and self.serial_port.is_open:
            self.text_log.insert(tk.END, "Calculating offsets based on RHS\n")
            self.serial_port.write(b'RHS\n')

    def submit_voltage(self):
        if self.serial_port and self.serial_port.is_open:
            voltage = self.voltage_var.get()
            print(voltage)
            if voltage > 8.3:
                self.serial_port.write(b'BATTERY_HIGH')
            elif (voltage <= 8.3) & (voltage > 7.7):
                self.serial_port.write(b'BATTERY_MEDIUM')
            elif (voltage <= 7.7) & (voltage > 7.45):
                self.serial_port.write(b'BATTERY_LOW')
            elif voltage < 7.45:
                self.serial_port.write(b'BATTERY_CRITICAL')
            
if __name__ == "__main__":
    root = tk.Tk()
    app = RoverControlApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
