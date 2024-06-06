#!/usr/bin/env bash

#Going to the target directory
cd /home/rhex/

#Downloading the robot controller
gdown https://drive.google.com/uc?id=1mtr-yEbGLTXMa2HJkLwIvsZYEt_7Wdnx

#Extracting the robot controller
tar -xzf robot_x64.tar.gz
#Removing the compressed file
rm robot_x64.tar.gz
#Renaming the extracted folder
mv robot_x64/ robot/

#Setting environment variables
export ROBOMETU_LINUX_DIR=/home/rhex/robot/RoboMETU.linux
export TRHEX_LINUX_DIR=/home/rhex/robot/TRHex.linux
export RHEX_API_DIR=/home/rhex/robot/rhex-api