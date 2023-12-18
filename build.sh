#!/bin/bash
set -e

cd ThirdParty
tp_ws_dir=$(pwd)

## Build Thirdparty Dependencies
# Install ArduPilot
cd $tp_ws_dir/ardupilot
git submodule update --init --recursive

# Install Arduopilot Gazebo Plugin
cd $tp_ws_dir/ardupilot_gazebo
# Check if build directory exists
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


# Set the default build type
# BUILD_TYPE=RelWithDebInfo
# catkin build
