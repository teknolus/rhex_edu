# RHex Educational - ROS2 and Gazebo Based Development Environment

<p style="color:red;">
This documentation is for educational purposes and may or may not include incorrect/incomplete information. We strive to write as comprehensively as possible, but it is not possible to include everything. Therefore, we would greatly appreciate your contributions to this repository with the knowledge you gain during your work.
</p>

<!------------------------------------------------------------------------>

## Table of Contents

- [Introduction](#introduction)
- [Key Features](#key-features)
- [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
- [Usage](#usage)
    - [Create Your Own Workspace](#create-your-own-workspace)
    - [Building The Workspace](#building-the-workspace)
    - [Running The Simulation](#running-the-simulation)
    - [Explore and Modify](#explore-and-modify)
- [TODO List](#todo-list)
- [Troubleshooting](#troubleshooting)
- [Acknowledgments](#acknowledgments)

<!------------------------------------------------------------------------>

## Introduction
Welcome to the RHex Educational repository! This project provides a comprehensive development environment for the RHex robot, leveraging the power of ROS2 (Robot Operating System 2) and Gazebo, a popular robot simulation tool. This repository aims to facilitate the learning and development of robotic applications in an educational setting.

<!------------------------------------------------------------------------>

## Key Features
- **ROS2 Integration:** Utilize the advanced capabilities of ROS2 for building robust and scalable robotic applications.

- **Gazebo Simulation:** Develop and test your RHex robot algorithms in a realistic simulation environment before deploying them on physical hardware.

- **Educational Resources:** Access a collection of tutorials, sample codes, and documentation designed to help students and educators get started with RHex, ROS2, and Gazebo.

<!------------------------------------------------------------------------>

## Getting Started

### Prerequisites
`Docker` and `VS Code` is required for building the environment and using packages.

#### Docker Installation:
1. Install [Docker Engine](https://docs.docker.com/engine/install/)

2. **For PCs with NVIDIA GPUs:** Make sure you are using official NVIDIA drivers. This can be done using `Software & Updates` app of Ubuntu. Go to `Additional Drivers` tab and choose the driver that is a `tested` one. Then, apply the changes (may need rebooting). You can confirm the correct installation using `sudo docker run --rm --runtime=nvidia --gpus all ubuntu nvidia-smi`

3. Make sure that you are working with [X11](https://beebom.com/how-switch-between-wayland-xorg-ubuntu/) not Wayland.

4. Allow X server connections from the container by running `xhost +local:docker` on the host machine. You might want to add this command to your bashrc using `echo 'xhost +local:docker' >> ~/.bashrc`

5. Create a docker usergroup and add yourself in it to be able use docker without sudo. The following [link](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user) will guide you.


#### VS Code Installation
1. Install [VS Code](https://code.visualstudio.com/)

2. In VS Code, from extensions tab, that can be found from the left-side panel, [install Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension pack

3. Install [Docker Extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) for VS Code (optional)

4. **For PCs without NVIDIA GPUs:** Remove `"--gpus", "all"` part from the following code block located in the file `rhex_edu/.devcontainer/devcontainer.json `

```
    "runArgs": [
        "--net=host",
        "--gpus", "all"
    ],
```

### Installation
Step-by-step instructions for setting up the development environment is provided below:

1. **Clone the Repository**
    - Start by cloning this repository to your local machine. Run `git clone https://github.com/teknolus/rhex_edu.git` command on your terminal where you want this repository to be in.

2. **Starting the Container**
    - Open the repository with VS Code by running `code rhex_edu` command on your terminal. To verify that you are in the correct folder, look for a notification from VS Code indicating that a devcontainer configuration file has been detected. You will see a prompt in the bottom right corner asking if you want to open the repository in a container.

    - Press `Ctrl+Shift+P` select `Rebuild/Reopen Container`. This will build the Docker environment that will be accessible from VS Code. First time **building the container will take a long time** however, after building once you will use **Ctrl+Shift+P** then **Reopen Container** to open the container since it was built previously.

    **Checking Installation** Try running rviz2 by opening a terminal inside VS Code then typing `rviz2` in that terminal and running it. You should be able to see the GUI appears.

3. **Install Dependencies (not necessary for now)**
    - Run `rosdep install --from-paths src --ignore-src -r -y` command inside the VS Code terminal (same terminal that you run `rviz2` command). All dependencies for ROS2 packages will be installed inside the container.

    - After that run `colcon build --symlink-install` command inside **rhex_ws** folder which should only have the **src** folder in it. For any problems that might occur check [Troubleshooting](#troubleshooting) Section.

<!------------------------------------------------------------------------>

## Usage

### Create Your Own Workspace
- In our workflow, it is required to work on the personal branch of your own because the `main` branch of the repository is for general purpose usage and it has to be working (should be executable without an error). To create your own environment, you can use the following commands with your name. This will push your own branch to the remote repository and will allow you to work on your project without any conflicts with others
```
git checkout -b dev/<your-name>
git push -u origin dev/<your-name>
```


### Building The Workspace
- To build the workspace run `colcon build --symlink-install` command inside the **rhex_ws** directory. After adding new files to the folders or after changing a code that can be compiled (C++), you should run this command to build the workspace. Here, `--symlink-install` is used to create symbolic links to the files instead of copying them which will allow us to change the file content and see the changes without rebuilding the workspace (pnly for Python).

- Be careful! Do not to run the `colcon build` command in the **src** directory of the workspace. Always run it inside of the **rhex_ws** directory. Which should have **src** folder in it. You can check it by running `ls` command in the terminal.


### Running The Simulation
To run the simulation and control the robot, we need to run four different commands in four different VS Code terminals.
- `ros2 launch rhex_gazebo start_sim.launch.py` Runs the simulation, summons RHex and prepares ROS nodes.
- `ros2 launch rhex_control start_controller_server.launch.py` Listens to the RHex controller and transfers the commands to Gazebo sim (Yes, it is not the controller even though it is called that).
- `start_rhex_supervisor.sh` Starts the actual controller of RHex
- `start_fltk_gui.sh` Starts GUI. One can calibrate the robot and start to operate using buttons under `Turbo` section. You can explore the GUI, it is an old one.

For the last two commands, we won't make an impromevent since they are irrelevant for our work. Instead we want to write our own simulation controller and GUI.

You can also use the command from `bringup` package to run the above commands at once:
```
ros2 launch rhex_bringup bringup_simulation_gui.launch.py
```

### Explore and Modify
- **rhex_gazebo** directory contains gazebo related files and worlds check README.md inside that folder.
- **rhex_control** directory contains the controller for the robot check README.md inside that folder.
- **rhex_msgs** directory contains the custom messages for the robot check README.md inside that folder.
- **rhex_description** directory contains the URDF files for the robot check README.md inside that folder.
- **rhex_bringup** directory contains the launch files for the robot check README.md inside that folder.

With the knowledge above you can modify files, or create your own to enhance your understanding and skills.

<!------------------------------------------------------------------------>

## Troubleshooting
There are common issues and their possible solutions listed below.

1. Issue
    - Solution
2. Issue
    - Solution

<!------------------------------------------------------------------------>

## TODO List
- Polish up the codes that are written before
- Write the guidelines from installation to the sample usage(**DONE**)
- Configure all the README.md files inside the directories so that they will be more educational
- Change the directory of the robot controller to rhex_control and change Dockerfile accordingly
- Configure the rhex_bringup directory
- Configure the rhex_gazebo directory for custom worlds
- Configure the rhex_msgs directory

<!------------------------------------------------------------------------>

## Acknowledgments
We would like to thank the contributors for their invaluable support and contributions to this project.
1.  🔥 Cem Önem
    - [Gitlab](https://gitlab.com/cemonem), [Github](https://github.com/cemonem)
2.  🔥 Osama Awad
    - [Gitlab](https://gitlab.com/usame_aw), [Github](https://github.com/usame-aw)
