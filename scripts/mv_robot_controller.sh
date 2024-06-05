#!/usr/bin/env bash

gdown https://drive.google.com/uc?id=1mtr-yEbGLTXMa2HJkLwIvsZYEt_7Wdnx

tar -xzf robot_x64.tar.gz
rm robot_x64.tar.gz

mv robot_x64/ robot/
mv robot/ /home/rhex/mnt/rhex_ws/src/rhex_control
rm -rf robot/

export ROBOMETU_LINUX_DIR=/home/${USERNAME}/robot/RoboMETU.linux
export TRHEX_LINUX_DIR=/home/${USERNAME}/robot/TRHex.linux
export RHEX_API_DIR=/home/${USERNAME}/robot/rhex-api
