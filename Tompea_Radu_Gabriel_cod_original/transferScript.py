import serial
import os
from influxdb_client import InfluxDBClient, Point
from dotenv import load_dotenv
import tkinter as tk
from tkinter import messagebox
import serial
import serial.tools.list_ports

# list available serial ports
ports = list(serial.tools.list_ports.comports())
port_list = "\n".join([f"{p.device}: {p.description}" for p in ports]) or "No serial ports found."

# pop up with available serial ports and ask for COM port
def get_com_port():
    while True:
        def on_ok():
            nonlocal com_port
            com_port = entry.get()
            root.destroy()
        def on_exit():
            root.destroy()
            os._exit(0)  # immediately exit the script

        com_port = None
        root = tk.Tk()
        root.title("Select COM Port")
        tk.Label(root, text=f"Detected serial ports:\n\n{port_list}\n\nEnter COM port (e.g., COM6):").pack(padx=10, pady=10)
        entry = tk.Entry(root)
        entry.pack(padx=10, pady=5)
        entry.focus()
        button_frame = tk.Frame(root)
        button_frame.pack(pady=10)
        tk.Button(button_frame, text="OK", command=on_ok).pack(side=tk.LEFT, padx=5)
        tk.Button(button_frame, text="Exit", command=on_exit).pack(side=tk.LEFT, padx=5)
        root.mainloop()
        # validate COM port
        available_ports = [p.device for p in ports]
        if com_port in available_ports:
            return com_port
        elif com_port is not None:
            tk.messagebox.showerror("Invalid Port", f"'{com_port}' is not a valid port. Please try again.")

selected_com_port = get_com_port()
if not selected_com_port:
    messagebox.showerror("Error", "No COM port selected. Exiting.")
    exit(1)

# load .env file
load_dotenv()

# read values from .env
INFLUXDB_URL = f"http://{os.getenv('DOCKER_INFLUXDB_INIT_HOST')}:{os.getenv('DOCKER_INFLUXDB_INIT_PORT')}"
INFLUXDB_TOKEN = os.getenv("DOCKER_INFLUXDB_INIT_ADMIN_TOKEN")
INFLUXDB_ORG = os.getenv("DOCKER_INFLUXDB_INIT_ORG")
INFLUXDB_BUCKET = os.getenv("DOCKER_INFLUXDB_INIT_BUCKET")

# serial port configuration (Windows)
ser = serial.Serial(selected_com_port, 9600)
print(f"Connected to {selected_com_port} at 9600 baud.")
# initialize InfluxDB client
client = InfluxDBClient(url=INFLUXDB_URL, token=INFLUXDB_TOKEN, org=INFLUXDB_ORG)
write_api = client.write_api()

start_influx = False

try:
    while True:
        line = ser.readline().decode('utf-8').strip()
        #print(line)
        if line:
            if line == "START INFLUX":
                start_influx = True
                print("Received START INFLUX")
            elif start_influx:
                write_api.write(bucket=INFLUXDB_BUCKET, org=INFLUXDB_ORG, record=line)
                print(f"Sent to InfluxDB: {line}")
                print(f"URL: {INFLUXDB_URL} | Org: {INFLUXDB_ORG} | Bucket: {INFLUXDB_BUCKET}")
                start_influx = False
except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
