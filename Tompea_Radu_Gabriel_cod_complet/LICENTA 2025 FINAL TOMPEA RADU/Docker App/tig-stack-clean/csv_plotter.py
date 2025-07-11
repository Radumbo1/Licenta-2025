import tkinter as tk
from tkinter import filedialog
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.widgets import Button
import math
import json
import os
import mplcursors

# open file to select JSON or CSV
root = tk.Tk()
root.withdraw()
file_path = filedialog.askopenfilename(
    title="Select JSON or CSV file",
    filetypes=[("JSON/CSV files", "*.json;*.csv"), ("All files", "*.*")]
)
if not file_path:
    print("No file selected.")
    exit()

ext = os.path.splitext(file_path)[1].lower()
if ext == ".json":
    with open(file_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    df = pd.DataFrame(data)
elif ext == ".csv":
    df = pd.read_csv(file_path, comment="#")
    rename_map = {
        "_time": "timestamp",
        "_field": "metric_name",
        "_value": "value"
    }
    df = df.rename(columns=rename_map)
    # only keep required columns if they exist
    required_csv_cols = ["timestamp", "metric_name", "value"]
    if not all(col in df.columns for col in required_csv_cols):
        print(f"CSV file must contain columns: {required_csv_cols} (after renaming)")
        exit()
    df = df[required_csv_cols]
else:
    print("Unsupported file type.")
    exit()

required_cols = {'timestamp', 'metric_name', 'value'}
if not required_cols.issubset(df.columns):
    print(f"File must contain columns: {required_cols}")
    exit()

df['timestamp'] = pd.to_datetime(df['timestamp'])

preferred_order = [
    "brake_sens", "IMU_Speed", "acc2", "accelX", "cooling", "current_sensor", "flowValue"
]

all_metrics = [
    "cooling", "brake_sens", "steer_sens", "acc1", "acc2", "wheel_spin_1", "wheel_spin_2",
    "motor_temp_1", "motor_temp_2", "control_temp_1", "control_temp_2", "serial_throttle_1",
    "serial_throttle_2", "inputV1_p", "inputV2_p", "phasecurrent1", "phasecurrent2",
    "susp_travel_FL", "susp_travel_FR", "susp_travel_BL", "susp_travel_BR", "ECU_control",
    "VRef_precharge", "meas_precharge", "current_sensor", "LV_state_of_charge", "ventValue",
    "pumpValue", "flowValue", "brakeEngaged", "soundPlaying", "R2D_Button_State",
    "airPlusValue", "airMinusValue", "prechgValue", "SDC_END", "Measure_Digital",
    "accelX", "accelY_", "roll", "pitch", "yaw", "IMU_Speed", "rawTemp1", "rawTemp2"
]

desired_order = preferred_order + [m for m in all_metrics if m not in preferred_order]

metrics = [m for m in desired_order if m in df['metric_name'].unique()]

df = df[df['metric_name'].isin(metrics)]

n_metrics = len(metrics)

plots_per_page = 2
n_pages = math.ceil(n_metrics / plots_per_page)
current_page = [0] 

fig, axes = plt.subplots(2, 1, figsize=(14, 10))
axes = axes.flatten()

def plot_page(page, fig, axes):
    start = page * plots_per_page
    end = min(start + plots_per_page, n_metrics)
    for i, ax in enumerate(axes):
        ax.clear()
        idx = start + i
        if idx < end:
            metric = metrics[idx]
            group = df[df['metric_name'] == metric]
            line, = ax.plot(group['timestamp'], group['value'], marker='o')
            ax.set_title(metric)
            ax.set_xlabel("Timestamp")
            ax.set_ylabel("Value")
            ax.grid(True)
            ax.set_visible(True)
            mplcursors.cursor(line, hover=True)
        else:
            ax.set_visible(False)
    fig.suptitle(f"Page {page+1} of {n_pages}")
    fig.canvas.draw_idle()

axprev = plt.axes([0.3, 0.01, 0.1, 0.05])
axnext = plt.axes([0.6, 0.01, 0.1, 0.05])
bnext = Button(axnext, 'Next')
bprev = Button(axprev, 'Previous')

def next_page(event):
    if current_page[0] < n_pages - 1:
        current_page[0] += 1
        plot_page(current_page[0], fig, axes)

def prev_page(event):
    if current_page[0] > 0:
        current_page[0] -= 1
        plot_page(current_page[0], fig, axes)

bnext.on_clicked(next_page)
bprev.on_clicked(prev_page)

plot_page(current_page[0], fig, axes)
plt.tight_layout(rect=[0, 0.08, 1, 1])
plt.show()