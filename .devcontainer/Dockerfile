FROM osrf/ros:humble-desktop-full

# Set non-interactive frontend to prevent interactive prompts during package installations
ARG DEBIAN_FRONTEND=noninteractive

# Set SHELL to use bash for the following RUN commands
# Define environment variable for default shell
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

RUN pip3 install gdown

# Add user, so the files created in the container are not owned by root
# Following operations needs 'sudo' to be installed

# Add a new user 'rhex' to use instead of 'root' for better security practices
# Configure user with no-password sudo access
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

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /usr/share/gazebo/setup.sh" >> ~/.bashrc

# Clean up APT cache to reduce the image size
RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*
