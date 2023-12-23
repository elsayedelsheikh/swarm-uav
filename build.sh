#!/bin/bash
set -e
ws_dir=$(pwd)

## Build Thirdparty Dependencies
# Install ArduPilot
cd $ws_dir/ThirdParty/ardupilot
git submodule update --init --recursive

# Install Arduopilot Gazebo Plugin
cd $ws_dir/ThirdParty/ardupilot_gazebo
if [[ -d "build" ]]; then
    echo "Build directory exists. Proceeding with the build process."
else
    echo "Build directory does not exist. Creating the directory."
    mkdir "build"
fi
cd build
cmake ..
make 
sudo make install

## Build the ROS Workspace
cd $ws_dir
catkin_make
# echo "source $ws_dir/devel/setup.bash" >> ~/.bashrc
# echo "export GAZEBO_MODEL_PATH=$ws_dir/ThirdParty/ardupilot_gazebo/models" >> ~/.bashrc
