import tkinter as tk
from subprocess import run

def send_command(command):
    run(command, shell=True)

root = tk.Tk()
root.title("ROS Controller")

tk.Button(root, text="Enable Simple Walker", command=lambda: send_command("ros2 param set /simple_walker simple_walker_enable True")).pack()
tk.Button(root, text="Sitting", command=lambda: send_command("ros2 param set /simple_walker state 1")).pack()
tk.Button(root, text="Standing", command=lambda: send_command("ros2 param set /simple_walker state 2")).pack()
tk.Button(root, text="Walking Mode 1", command=lambda: send_command("ros2 param set /simple_walker state 3")).pack()
tk.Button(root, text="Walking Mode 2", command=lambda: send_command("ros2 param set /simple_walker state 4")).pack()
tk.Button(root, text="Turn Right", command=lambda: send_command("ros2 param set /simple_walker state 5")).pack()
tk.Button(root, text="Turn Left", command=lambda: send_command("ros2 param set /simple_walker state 6")).pack()

root.mainloop()
