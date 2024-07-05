#!/usr/bin/env bash

# Need to make sure that the script is executable
chmod +x /home/rhex/mnt/rhex_ws/src/rhex_control/scripts/controller_server_network.py

colcon build --symlink-install --packages-select rhex_description rhex_gazebo rhex_control rhex_bringup
source install/setup.bash

echo "source /home/rhex/mnt/rhex_ws/install/setup.bash" >> ~/.bashrc
