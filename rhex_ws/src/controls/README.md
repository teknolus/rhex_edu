
# RHex Controller Android GUI 

## Introduction
This project integrates robotics and software front-end engineering to enhance the RHex simulation controller GUI.

## Team Members
- **Leen Said**
 - **Hamza Awad**

## Project Overview
This project aims to develop an Android GUI for controlling a RHex robot simulation by enabling communication between an Android device and a host PC running a containerized RHex simulation. A Flask server is set up inside the container, with modifications to the Dockerfile to include necessary precompiled binaries, and port 5001 is opened for communication. The host machine's firewall is configured to allow traffic through this port. By connecting the host PC, container, and Android device to a common network, and running the server and Gazebo workspace within the container, users can control the simulation by entering the host PC's IP address on the Android app.

## Technologies Used
- **Languages**: C++, Python, Java
- **Libraries**: Flask, GStreamer, GLib, OpenSSL, Argon2
- **Tools**: Docker, Android Studio, Gazebo


## Flask Server (server.py)
- The server provides endpoints to run control commands and compile code. This allows users to interact with the robot control system via HTTP requests. The server starts and listens on port 5001.

- Provides an endpoint (/run/\<command>) to execute control commands via HTTP POST requests.

  

## Control Client (control_client.cc)
- The script allows for remote control of a robot or system by sending commands to a control server. The commands specified via the terminal or through an Android GUI connected to a Flask server.

- The run_control_client function in server.py changes the directory to the control_client script's location and executes the specified command using the subprocess module. For example, if the command were "connect" the function would execute the following in the terminal:
  ```bash
    ./control_client connect
  ```


## Control Server (control_server.cc)
- This script implements a server that handles robot control commands received over a TCP connection using the RHexAPI. 
- The `handleClient` function processes incoming commands such as CONNECT, STAND, SIT, DISCONNECT, WALK, and movement directions (FORWARD, BACKWARD, STOP, RIGHT, LEFT) by interacting with the RHexAPI.
-  Commands are sent to the robot, and the robot's responses are returned to the client.


### Note: This project uses precompiled c++ object files (`control_server` and `control_client`), the original c++ files are provided in case any modifications are necessary. They c++ files can be compiled using the following command in `src/controls/`:
  ```bash
  g++ -o object_file_name file_name.cc -I../../../../robot/rhex-api/include -I/usr/include/gstreamer-1.0 -I/usr/include/glib-2.0 -I/usr/lib/x86_64-linux-gnu/glib-2.0/include -L../../../../robot/rhex-api/lib -L/usr/lib/x86_64-linux-gnu -lrhexapi -lgstreamer-1.0 -lgobject-2.0 -lglib-2.0 -lssl -lcrypto -lgstvideo-1.0 -lgstrtp-1.0 -largon2
  ```

## How to Use the App/Code
### Setup Instructions
Clone the repository and follow the instructions to build and open the container.


### Running the Application
***Outside The Container***
- Configure Host to allow communication on port 5001:
  ```sh
  sudo ufw enable
  sudo ufw allow 5001/tcp
  sudo ufw reload
  ```
***Inside The Container***
- Connect devices to a common network.
- Start the Flask server(communicates with the user's phone):
  ```sh
  python3 src/flask_server/server.py
  ```
- Run the client server(connects the flask server with the robot so that it can receive commands):
  ```sh
  ./src/controls/control_server
  ```
- Run the simulation:
  ```sh
  ros2 launch rhex_gazebo start_sim.launch.py 
  ros2 launch rhex_control start_controller_server.launch.py
  start_rhex_supervisor.sh
  ```




***On Android***
1. Open the app on your Android device.
2. Enter the host machine's IP address
3. Voila!



