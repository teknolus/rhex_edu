
# RHex Controller Android GUI 

## Introduction
This project integrates robotics and software front-end engineering to enhance the RHex simulation controller GUI.

## Team Members
- **Leen Said**
 - **Hamzeh Awad**

## Project Overview
This project aims to develop an Android GUI for controlling a RHex robot simulation by enabling communication between an Android device and a host PC running a containerized RHex simulation. A Flask server is set up inside the container, with modifications to the Dockerfile to include necessary precompiled binaries, and port 5001 is opened for communication. The host machine's firewall is configured to allow traffic through this port. By connecting the host PC, container, and Android device to a common network, and running the server and Gazebo workspace within the container, users can control the simulation by entering the host PC's IP address on the Android app.

## Technologies Used
- **Languages**: C++, Python, Java
- **Libraries**: Flask, GStreamer, GLib, OpenSSL, Argon2
- **Tools**: Docker, Android Studio, Gazebo

## Overview

```server.py```

The server.py script sets up a Flask server to interface with a RHex robot. It handles HTTP requests to control the robot, such as connecting, standing, sitting, walking, and calibrating. The script integrates ROS2 for image processing using the YOLO model for object detection. Images are processed and served via the Flask server, allowing for real-time monitoring and control of the robot.

```pyrhexapi.cpp```

The pyRiminate the need for client_controller and client_server C++ files.

hexapi.cpp script uses pybind11 to create Python bindings for the RHexAPI, enabling Python programs to interact with the RHex robot's functionalities. It exposes various classes and functions such as IMU data, GPS data, movement commands, and calibration processes, allowing Python developers to control the robot, retrieve sensor data, and manage its operational modes directly from Python code.

```setup.py```

The setup.py script configures the build process for the pyrhexapi module using setuptools and CMake. It defines CMakeExtension and CMakeBuild classes to manage building the C++ extension. The script sets up CMake arguments, handles different build configurations, and ensures compatibility across various platforms. This setup allows for seamless building and installation of the module in a Python environment.

## How to Use the App/Code
### Setup Instructions
Clone the repository and follow the instructions to build and open the container.


### Running the Application
***Outside The Container***
- Configure Host to allow communication on port 5001:
  ```sh
  sudo ufw enable
  sudo ufw allow 5001/tcp
  ```
***Inside The Container***
- Set the RHEX_API_DIR environment variable to the path of gui_controls. The default command is:
  ```bash
  export RHEX_API_DIR=/home/rhex/mnt/rhex_ws/src/gui_controls
  ```
- Navigate to the gui_controls/build folder and run the following commands to build the project:
  ```bash
  cd /home/rhex/mnt/rhex_ws/src/gui_controls/build
  cmake ..
  make
  ```
These commands will configure and compile the gui_controls project. A shared object file (".so") should appear in the `gui_controls/src` directory.
Ensure that the path /home/rhex/mnt/rhex_ws/src/gui_controls matches the actual location of the gui_controls directory on your system. If the directory is located elsewhere, adjust the path accordingly.

- Connect devices to a common network.
- Go back to `rhex_ws` and start the Flask server:
  ```sh
  cd ../../..
  python3 src/gui_controls/src/server.py 
  ```
  #### Note: This script is designed to execute automatically within the rhex_edu container environment on any host machine. For scenarios where you prefer to run the script outside this environment, you can utilize the -cp and the terminal will prompt you to input your custom path.
   ```sh
    python3 src/gui_controls/src/server.py -cp
  ```
- In another terminal, run the simulation and controller:
  ```sh
  ros2 launch rhex_gazebo start_sim.launch.py 
  ros2 launch rhex_control start_controller_server.launch.py
  start_rhex_supervisor.sh
  ```

  ***On Android***
1. Open the app on your Android device.
2. Enter the host machine's IP address
3. Voila!

### Simulating Android App Button Press:
To simulate an Android app button press, run the following command in a terminal outside the container:

```bash 
  curl -X POST http://<machine-ip-address>:5001/control/<command>
  ```
Replace <machine-ip-address> with the IP address of the machine running the server and <command> with one of the available commands listed below.

Available commands:

- connect
- sit
- stand
- calibrate
- walk
- forward
- backward
- right
- left
- stop
- disconnect<br>
For testing from within the Docker container, you can use either the previous command with the appropriate IP address or the following command:
```bash 
  curl -X POST http://127.0.0.1:5001/control/<command>
  ```




### Deploying Code on RHex Robot
- Change the IP Address in client_server.cc to match that of the RHex Robot.

- Ensure you update the ROS topic names in server.py to match the actual topic names used by the RHex robot. Replace the following lines with the appropriate topic names:
  ```
    super().__init__('image_processor')
      self.image_subscription = self.create_subscription(
          Image,
          '/depth_camera/image_raw',  # Update this topic name
          self.image_callback,
          10
    )
  ```
- Note: These commands are unnecessary when operating the actual RHex robot:
  ```
  ros2 launch rhex_gazebo start_sim.launch.py 
  ros2 launch rhex_control start_controller_server.launch.py
  start_rhex_supervisor.sh
  ```



