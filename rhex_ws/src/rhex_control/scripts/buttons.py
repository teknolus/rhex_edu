import tkinter as tk
from tkinter import ttk
from subprocess import run

def send_command(command):
    run(command, shell=True)



def update_label(var, label):
    label.config(text=f"{var.get():.2f}")

def create_param_frame(parent, label_text, min_val, max_val, var_list, button_command):
    frame = tk.Frame(parent, padx=10, pady=10)
    frame.pack(side=tk.TOP, fill=tk.X)
    tk.Label(frame, text=label_text).pack()

    entry_vars = []
    for _ in range(6):  # Assuming you need 6 entry fields
        var = tk.DoubleVar(value=min_val)
        entry_frame = tk.Frame(frame)
        entry_frame.pack(fill=tk.X, pady=2)
        
        entry = tk.Entry(entry_frame, textvariable=var)
        entry.pack(side=tk.LEFT, fill=tk.X, expand=True)

        value_label = tk.Label(entry_frame, text=f"{var.get():.2f}")
        value_label.pack(side=tk.RIGHT, padx=10)

        # Update the value label when the entry field changes
        entry.bind("<KeyRelease>", lambda event, v=var, l=value_label: update_label(v, l))

        entry_vars.append(var)
    var_list.extend(entry_vars)
    tk.Button(frame, text=f"Set {label_text}", command=button_command).pack(pady=5)

root = tk.Tk()
root.title("ROS Controller")

# Create frames for different functionalities
control_frame = tk.Frame(root, padx=10, pady=10)
control_frame.grid(row=0, column=0, sticky='ns')  # Column for Simple Walker and Test Robot controls

param_frame = tk.Frame(root, padx=10, pady=10)
param_frame.grid(row=0, column=1, sticky='nsew')  # Column for parameter sliders

# Create frames for Simple Walker and Test Robot buttons
walker_frame = tk.Frame(control_frame, padx=10, pady=10)
walker_frame.grid(row=0, column=0, sticky='ns')  # Vertical column for Simple Walker

# Add Simple Walker buttons
walker_buttons = [
    ("Enable Walker", "ros2 param set /walker walker_enable True"),
    ("Disable Walker", "ros2 param set /walker walker_enable False"),
    ("Sitting Walker", "ros2 param set /walker state 1"),
    ("Standing Walker", "ros2 param set /walker state 2"),
    ("Walking Walker", "ros2 param set /walker state 3"),
    ("Enable Optimizer", "ros2 param set /optimizer_node optimizer_enable True"),
    ("Disable Optimizer", "ros2 param set /optimizer_node optimizer_enable False"),
]

for idx, (text, command) in enumerate(walker_buttons):
    tk.Button(walker_frame, text=text, command=lambda cmd=command: send_command(cmd)).grid(row=idx, column=0, pady=2, sticky='ew')



root.mainloop()
