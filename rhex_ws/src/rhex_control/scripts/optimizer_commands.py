import tkinter as tk
from tkinter import ttk
from subprocess import run

def send_command(command):
    run(command, shell=True)

def send_cmd_tau():
    tau_values = [tau_var.get() for tau_var in tau_vars]
    tau_command = f"ros2 param set /optimizer_node cmd_tau \"[{', '.join(map(str, tau_values))}]\""
    send_command(tau_command)

def send_cmd_kp():
    kp_values = [kp_var.get() for kp_var in kp_vars]
    kp_command = f"ros2 param set /optimizer_node cmd_kp \"[{', '.join(map(str, kp_values))}]\""
    send_command(kp_command)

def send_cmd_kd():
    kd_values = [kd_var.get() for kd_var in kd_vars]
    kd_command = f"ros2 param set /optimizer_node cmd_kd \"[{', '.join(map(str, kd_values))}]\""
    send_command(kd_command)

def send_cmd_period():
    period_value = period_var.get()
    period_command = f"ros2 param set /optimizer_node  period {period_value}"
    send_command(period_command)

def send_cmd_delta_t_s(robot_name):
    delta_t_s_value = delta_t_s_vars[robot_name].get()
    delta_t_s_command = f"ros2 param set /{robot_name} delta_t_s {delta_t_s_value}"
    send_command(delta_t_s_command)

def send_cmd_delta_phi_s(robot_name):
    delta_phi_s_value = delta_phi_s_vars[robot_name].get()
    delta_phi_s_command = f"ros2 param set /{robot_name} delta_phi_s {delta_phi_s_value}"
    send_command(delta_phi_s_command)

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

optimizer_node_frame = tk.Frame(control_frame, padx=10, pady=10)
optimizer_node_frame.grid(row=0, column=0, sticky='ns')  # Vertical column for Test Robot


# Add optimizer node buttons
optimizer_node_buttons = [
    ("Enable Optimizer Node", "ros2 param set /optimizer_node optimizer_node_enable True"),
    ("Disable", "ros2 param set /optimizer_node optimizer_node_enable False"),
    ("Sitting", "ros2 param set /optimizer_node state 1"),
    ("Standing", "ros2 param set /optimizer_node state 2"),
    ("Walking", "ros2 param set /optimizer_node state 3"),
    ("Running", "ros2 param set /optimizer_node state 4"),
    ("Turning", "ros2 param set /optimizer_node state 5"),
    ("Turning Left", "ros2 param set /optimizer_node state 6"),
    ("Walk with Period", "ros2 param set /optimizer_node state 7"),
    ("Walk and turn", "ros2 param set /optimizer_node state 8")
]

for idx, (text, command) in enumerate(optimizer_node_buttons):
    tk.Button(optimizer_node_frame, text=text, command=lambda cmd=command: send_command(cmd)).grid(row=idx, column=0, pady=2, sticky='ew')

# Adding period entry next to Custom State Test Robot button
period_var = tk.DoubleVar(value=0.0)
tk.Label(optimizer_node_frame, text="Period:").grid(row=len(optimizer_node_buttons), column=0, pady=2, sticky='ew')
period_entry = tk.Entry(optimizer_node_frame, textvariable=period_var)
period_entry.grid(row=len(optimizer_node_buttons), column=1, pady=2, sticky='ew')
tk.Button(optimizer_node_frame, text="Set Period", command=send_cmd_period).grid(row=len(optimizer_node_buttons)+1, column=0, columnspan=2, pady=2, sticky='ew')

# Adding sliders for delta_t_s and delta_phi_s
delta_t_s_vars = {
    "optimizer_node": tk.DoubleVar(value=0.0)
}
delta_phi_s_vars = {
    "optimizer_node": tk.DoubleVar(value=0.0)
}

# Test Robot Sliders
tk.Label(optimizer_node_frame, text="Delta T_s:").grid(row=len(optimizer_node_buttons)+2, column=0, pady=2, sticky='ew')
delta_t_s_slider_optimizer_node = tk.Scale(optimizer_node_frame, from_=-0.4, to=0.4, resolution=0.01, orient=tk.HORIZONTAL, variable=delta_t_s_vars["optimizer_node"])
delta_t_s_slider_optimizer_node.grid(row=len(optimizer_node_buttons)+2, column=1, pady=2, sticky='ew')
tk.Button(optimizer_node_frame, text="Set Delta T_s", command=lambda: send_cmd_delta_t_s("optimizer_noode")).grid(row=len(optimizer_node_buttons)+3, column=0, columnspan=2, pady=2, sticky='ew')

tk.Label(optimizer_node_frame, text="Delta Phi_s:").grid(row=len(optimizer_node_buttons)+4, column=0, pady=2, sticky='ew')
delta_phi_s_slider_optimizer_node = tk.Scale(optimizer_node_frame, from_=-0.4, to=0.4, resolution=0.01, orient=tk.HORIZONTAL, variable=delta_phi_s_vars["optimizer_node"])
delta_phi_s_slider_optimizer_node.grid(row=len(optimizer_node_buttons)+4, column=1, pady=2, sticky='ew')
tk.Button(optimizer_node_frame, text="Set Delta Phi_s", command=lambda: send_cmd_delta_phi_s("optimizer_node")).grid(row=len(optimizer_node_buttons)+5, column=0, columnspan=2, pady=2, sticky='ew')


# Create frames for each parameter
tau_vars = []
create_param_frame(param_frame, "cmd_tau Values:", -10, 10, tau_vars, send_cmd_tau)

kp_vars = []
create_param_frame(param_frame, "cmd_kp Values:", 0, 30, kp_vars, send_cmd_kp)

kd_vars = []
create_param_frame(param_frame, "cmd_kd Values:", 0, 2, kd_vars, send_cmd_kd)

# Configure grid weights to make the layout responsive
root.grid_columnconfigure(1, weight=1)  # Make the parameter column expand
root.grid_rowconfigure(0, weight=1)     # Make the row expand

root.mainloop()
