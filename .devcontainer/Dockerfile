FROM osrf/ros:humble-desktop-full

# Set non-interactive frontend to prevent interactive prompts during package installations
ARG DEBIAN_FRONTEND=noninteractive

# Set SHELL to use bash for the following RUN commands
SHELL ["/bin/bash", "-c"]
ENV SHELL=/bin/bash

# Update system and install necessary packages in a single RUN statement to reduce image layers
RUN apt-get update -y && \
    apt-get install -qq -y --no-install-recommends \
        build-essential \
        sudo \
        git \
        cmake \
        python3-pip \
        ros-humble-gazebo-* -y

RUN pip3 install gdown flask

# Add user, so the files created in the container are not owned by root
ARG USERNAME=rhex
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a group and user with specified UID/GID and add user to sudoers with no password entry
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Set the default user to the newly created user
USER $USERNAME

WORKDIR /home/${USERNAME}
RUN gdown https://drive.google.com/uc?id=1mtr-yEbGLTXMa2HJkLwIvsZYEt_7Wdnx
RUN tar -xzf robot_x64.tar.gz && \
    rm robot_x64.tar.gz
RUN mv robot_x64/ robot/

ENV ROBOMETU_LINUX_DIR=/home/${USERNAME}/robot/RoboMETU.linux
ENV TRHEX_LINUX_DIR=/home/${USERNAME}/robot/TRHex.linux
ENV RHEX_API_DIR=/home/${USERNAME}/robot/rhex-api

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# Expose port 5000
EXPOSE 5000
