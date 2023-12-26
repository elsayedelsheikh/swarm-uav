#!/bin/bash
set -e
ws_dir=$(pwd)

## Build Thirdparty Dependencies
# Build ArduPilot
echo "Building ArduPilot"
cd $ws_dir/ThirdParty/ardupilot
git submodule update --init --recursive
echo "Submodules updated"

# Install Arduopilot Gazebo Plugin
echo "Building ArduPilot Gazebo Plugin"
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
echo "ArduPilot Gazebo Plugin Installed"

# Build ORB_SLAM2
echo "Building ORB_SLAM2"
cd $ws_dir/ThirdParty/ORB_SLAM2_NOETIC

echo "Building Pangolin Dependency"
cd Pangolin
mkdir build
cd build
cmake ..
make -j
echo "Pangolin Dependency Installed"

echo "Building ORB_SLAM2"
cd $ws_dir/ThirdParty/ORB_SLAM2_NOETIC
chmod +x build.sh
./build.sh
echo "ORB_SLAM2 Installed"
echo "Building ORB_SLAM2 ROS Examples"
chmod +x build_ros.sh
./build_ros.sh
echo "ORB_SLAM2 ROS Examples Installed"

## Build the ROS Workspace
echo "Building the ROS Workspace"
cd $ws_dir
catkin_make
## Add the ROS Workspace to the bashrc
echo "Setting up the ROS Workspace"
echo "source $ws_dir/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$ws_dir/ThirdParty/ardupilot_gazebo/models" >> ~/.bashrc
echo "source export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$ws_dir/ORB_SLAM2_NOETIC/Examples/ROS" >> ~/.bashrc
echo "Done"

