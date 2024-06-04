#!/usr/bin/env bash

sudo apt update
rosdep update
rosdep install -i --from-path src --rosdistro humble -y

# Clean up APT cache to reduce the image size
sudo apt-get clean
sudo rm -rf /var/lib/apt/lists/*

colcon build --symlink-install --packages-select rhex_description rhex_gazebo
source install/setup.bash

echo "source /home/rhex/mnt/rhex_ws/install/setup.bash" >> ~/.bashrc