import tkinter as tk
from tkinter import ttk
from subprocess import run

def send_command(command):
    run(command, shell=True)

def send_cmd_tau():
    tau_values = [tau_var.get() for tau_var in tau_vars]
    tau_command = f"ros2 param set /test_robot cmd_tau \"[{', '.join(map(str, tau_values))}]\""
    send_command(tau_command)

def send_cmd_kp():
    kp_values = [kp_var.get() for kp_var in kp_vars]
    kp_command = f"ros2 param set /test_robot cmd_kp \"[{', '.join(map(str, kp_values))}]\""
    send_command(kp_command)

def send_cmd_kd():
    kd_values = [kd_var.get() for kd_var in kd_vars]
    kd_command = f"ros2 param set /test_robot cmd_kd \"[{', '.join(map(str, kd_values))}]\""
    send_command(kd_command)

def update_label(var, label):
    label.config(text=f"{var.get():.2f}")

root = tk.Tk()
root.title("ROS Controller")

# Create frames for different functionalities
control_frame = tk.Frame(root, padx=10, pady=10)
control_frame.grid(row=0, column=0, sticky='ns')  # Column for Simple Walker and Test Robot controls

param_frame = tk.Frame(root, padx=10, pady=10)
param_frame.grid(row=0, column=1, sticky='nsew')  # Column for parameter sliders

# Create frames for Simple Walker and Test Robot buttons
simple_walker_frame = tk.Frame(control_frame, padx=10, pady=10)
simple_walker_frame.grid(row=0, column=0, sticky='ns')  # Vertical column for Simple Walker

test_robot_frame = tk.Frame(control_frame, padx=10, pady=10)
test_robot_frame.grid(row=0, column=1, sticky='ns')  # Vertical column for Test Robot

# Add Simple Walker buttons
simple_walker_buttons = [
    ("Enable Simple Walker", "ros2 param set /simple_walker simple_walker_enable True"),
    ("Disable Simple Walker", "ros2 param set /simple_walker simple_walker_enable False"),
    ("Sitting Simple Walker", "ros2 param set /simple_walker state 1"),
    ("Standing Simple Walker", "ros2 param set /simple_walker state 2"),
    ("Walking Simple Walker", "ros2 param set /simple_walker state 3"),
    ("Running Simple Walker", "ros2 param set /simple_walker state 4"),
    ("Turning Right Simple Walker", "ros2 param set /simple_walker state 5"),
    ("Turning Left Simple Walker", "ros2 param set /simple_walker state 6")
]

for idx, (text, command) in enumerate(simple_walker_buttons):
    tk.Button(simple_walker_frame, text=text, command=lambda cmd=command: send_command(cmd)).grid(row=idx, column=0, pady=2, sticky='ew')

# Add Test Robot buttons
test_robot_buttons = [
    ("Enable Test Robot", "ros2 param set /test_robot simple_walker_enable True"),
    ("Disable Test Robot", "ros2 param set /test_robot simple_walker_enable False"),
    ("Sitting Test Robot", "ros2 param set /test_robot state 1"),
    ("Standing Test Robot", "ros2 param set /test_robot state 2"),
    ("Walking Test Robot", "ros2 param set /test_robot state 3"),
    ("Running Test Robot", "ros2 param set /test_robot state 4"),
    ("Turning Right Test Robot", "ros2 param set /test_robot state 5"),
    ("Turning Left Test Robot", "ros2 param set /test_robot state 6")
]

for idx, (text, command) in enumerate(test_robot_buttons):
    tk.Button(test_robot_frame, text=text, command=lambda cmd=command: send_command(cmd)).grid(row=idx, column=0, pady=2, sticky='ew')

# Create a function to create sliders and labels for parameters
def create_param_frame(parent, label_text, min_val, max_val, var_list, button_command):
    frame = tk.Frame(parent, padx=10, pady=10)
    frame.pack(side=tk.TOP, fill=tk.X)
    tk.Label(frame, text=label_text).pack()
    for _ in range(6):  # Assuming you need 6 sliders
        var = tk.DoubleVar(value=min_val)
        slider_frame = tk.Frame(frame)
        slider_frame.pack(fill=tk.X, pady=2)
        
        slider = ttk.Scale(slider_frame, from_=min_val, to=max_val, orient='horizontal', variable=var)
        slider.pack(side=tk.LEFT, fill=tk.X, expand=True)
        
        value_label = tk.Label(slider_frame, text=f"{var.get():.2f}")
        value_label.pack(side=tk.RIGHT, padx=10)

        # Update the value label when the slider moves
        slider.bind("<Motion>", lambda event, v=var, l=value_label: update_label(v, l))
        
        var_list.append(var)
    tk.Button(frame, text=f"Set {label_text}", command=button_command).pack(pady=5)

# Create frames for each parameter
tau_vars = []
create_param_frame(param_frame, "cmd_tau Values (0-10):", -10, 10, tau_vars, send_cmd_tau)

kp_vars = []
create_param_frame(param_frame, "cmd_kp Values (0-30):", 0, 30, kp_vars, send_cmd_kp)

kd_vars = []
create_param_frame(param_frame, "cmd_kd Values (0-2):", 0, 2, kd_vars, send_cmd_kd)

# Configure grid weights to make the layout responsive
root.grid_columnconfigure(1, weight=1)  # Make the parameter column expand
root.grid_rowconfigure(0, weight=1)     # Make the row expand

root.mainloop()
