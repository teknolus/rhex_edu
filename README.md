# RHex Educational - ROS2 and Gazebo Based Development Environment



## Table of Contents

- [Introduction](#introduction)
- [Key Features](#key-features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [Usage](#usage)
- [TODO List](#todo-list)
- [Troubleshooting](#troubleshooting)
- [Acknowledgments](#acknowledgments)


## Introduction
Welcome to the RHex Educational repository! This project provides a comprehensive development environment for the RHex robot, leveraging the power of ROS2 (Robot Operating System 2) and Gazebo, a popular robot simulation tool. This repository aims to facilitate the learning and development of robotic applications in an educational setting.


## Key Features
- **ROS2 Integration:** Utilize the advanced capabilities of ROS2 for building robust and scalable robotic applications.
- **Gazebo Simulation:** Develop and test your RHex robot algorithms in a realistic simulation environment before deploying them on physical hardware.
- **Educational Resources:** Access a collection of tutorials, sample codes, and documentation designed to help students and educators get started with RHex, ROS2, and Gazebo.


## Getting Started

### Prerequisites
**Docker and VS Code** is required for building the environment and using packages.
#### Docker and VS Code Installation:
1. First make sure that you are working with [X11](https://beebom.com/how-switch-between-wayland-xorg-ubuntu/) not Wayland. 
2. Install [Docker Engine](https://docs.docker.com/engine/install/).
3. [Make Docker non-root user](https://docs.docker.com/engine/install/linux-postinstall/#manage-docker-as-a-non-root-user). 
4. Allow Docker to access X11 sockets by running `xhost +local:docker` command on your terminal
5. Install [VS Code](https://code.visualstudio.com/).
6. In VS Code, from extensions tab that can be found from the left-side panel install [Remote Development Extension Pack](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.vscode-remote-extensionpack).
7. Install [Docker Extension](https://marketplace.visualstudio.com/items?itemName=ms-azuretools.vscode-docker) for VS Code. 


### Installation
Step-by-step instructions for setting up the development environment is provided below:
#### **Installation Steps:**
1. **Clone the Repository:** Start by cloning this repository to your local machine.
- Run `git clone https://gitlab.com/robot3852119/rhex_edu.git` command on your terminal where you want this repository to be in. For example mine is inside `/home/repos` directory.
2. **Running the Container:**
- **Open the Repository:** Open the repository with VS Code by running `code /home/repos/rhex_edu` command on your terminal. How to know whether you are in the correct folder is VS Code will notifiy you that it detected a devcontainer configuration file and will ask you to open the repository in a container on the right below corner. Then proceed to the next step.
- **For non-Nvidia GPUs:** If you don't have a Nvidia GPU or your computer has CPU embedded graphics card on your computer check **README.md** file in the **.devcontainer** directory for necessary changes.
- **Rebuild/Reopen Container:** After cloning the repository and finishing the installation of Docker and VS Code open your VS Code and open the ***rhex_edu*** folder. Then press Ctrl+Shift+P select **Rebuild/Reopen Container**. This will build the Docker environment that will be accessible from VS Code. First time building the container will take a long time however, after building once you will use Ctrl+Shift+P then **Reopen Container** to open the container since it was built previously. 
- **Checking Installation:** Try running gazebo by opening a terminal inside VS Code then typing `rviz2` in that terminal and running it, you should see the GUI open.
3. **Install Dependencies:** 
- Run `rosdep install --from-paths src --ignore-src -r -y` command inside the VS Code terminal (same terminal that you run `rviz2` command). All dependencies for ROS2 packages will be installed inside the container.
- After that run `colcon build --symlink-install` command inside **rhex_ws** folder which should only have the **src** folder in it. For any problems that might occur check [Troubleshooting](#troubleshooting) Section.
4. **Explore and Modify:** 
- **rhex_gazebo** directory contains gazebo related files and worlds check README.md inside that folder.
- **rhex_control** directory contains the controller for the robot check README.md inside that folder.
- **rhex_msgs** directory contains the custom messages for the robot check README.md inside that folder.
- **rhex_description** directory contains the URDF files for the robot check README.md inside that folder.
- **rhex_bringup** directory contains the launch files for the robot check README.md inside that folder.

With the knowledge above you can modify files, or create your own to enhance your understanding and skills.


## Usage
### Building The Workspace
- Be careful! Do not to run the `colcon build` command in the **src** directory of the workspace. Always run it outside of the **rhex_ws** directory. Which should have **src** folder in it. You can check it by running `ls` command in the terminal.
- To build the workspace run `colcon build --symlink-install` command inside the **rhex_ws** directory. After adding new files to the folders you should run this command to build the workspace. `--symlink-install` is used to create symbolic links to the files instead of copying them which will allow us to change the file content and see the changes without rebuilding the workspace.
---
After building the workspace you need to run `source /home/rhex/mnt/rhex_ws/install/setup.bash` to activate your workspace. Or you can add this line to the `~/.bashrc`, so that workspace will be automatically activated when a new terminal is opened. To achieve this run the following code in the terminal:
```
echo "source /home/rhex/mnt/rhex_ws/install/setup.bash" >> ~/.bashrc
```
---

### Running The Simulation 
- To run the simulation try running `ros2 launch rhex_bringup start_sim.launch.py`. Which will launch all the necessary nodes.
- More will be added...


## Troubleshooting
There are common issues and their possible solutions listed below.

1. Issue
    - Solution
2. Issue
    - Solution


## TODO List
- Polish up the codes that are written before
- Write the guidelines from installation to the sample usage(**DONE**)
- Configure all the README.md files inside the directories so that they will be more educational
- Change the directory of the robot controller to rhex_control and change Dockerfile accordingly
- Configure the rhex_bringup directory
- Configure the rhex_gazebo directory for custom worlds
- Configure the rhex_msgs directory


## Acknowledgments
We would like to thank the contributors for their invaluable support and contributions to this project.
1.  🔥 Cem Önem
    - [Gitlab](https://gitlab.com/cemonem), [Github](https://github.com/cemonem)
2.  🔥 Osama Awad
    - [Gitlab](https://gitlab.com/usame_aw), [Github](https://github.com/usame-aw)