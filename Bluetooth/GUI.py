import tkinter as tk
from tkinter import messagebox, filedialog
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

        self.ping_button = tk.Button(self.control_frame, text="Ping", command=self.send_ping, state='disabled')
        self.ping_button.grid(row=0, column=3, padx=5)

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
            self.ping_button.config(state='normal')
            self.running = True
            threading.Thread(target=self.read_serial, daemon=True).start()

            # Send ping after connection
            self.send_ping()

            messagebox.showinfo("Connection", f"Connected to {port}")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def send_start(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'start\n')
            messagebox.showinfo("Start Command", "Start command sent!")

    def send_stop(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.write(b'stop\n')
            messagebox.showinfo("Stop Command", "Stop command sent!")

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
                    else:
                        self.data_log.append(line.split(','))
                        self.text_log.insert(tk.END, line + '\n')
                    self.text_log.see(tk.END)
            time.sleep(0.05)

    def save_log(self):
        if self.data_log:
            filepath = filedialog.asksaveasfilename(defaultextension=".csv",
                                                    filetypes=[("CSV files", "*.csv")])
            if filepath:
                with open(filepath, 'w', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow(["DistanceLeft", "DistanceRight", "SteeringAngle"])
                    writer.writerows(self.data_log)
                messagebox.showinfo("Saved", "Log saved successfully.")

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
