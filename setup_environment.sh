#!/bin/bash
set -e
ws_dir=$(pwd)

# Function to echo_green in green
echo_green() {
    echo -e "\e[32m$1\e[0m"  # \e[32m sets the text color to green, \e[0m resets the color
}

## Build Thirdparty Dependencies
# Build ArduPilot
echo_green "Building ArduPilot"
max_retries=3
retry_count=0
update_submodules() {
    git submodule update --init --recursive
}
while [ $retry_count -lt $max_retries ]; do
    cd "$ws_dir/ThirdParty/ardupilot"
    update_submodules

    # Check if submodule update was successful
    if [ $? -eq 0 ]; then
        echo_green "Submodules updated successfully"
        break
    else
        echo_green "Submodule update failed. Retrying..."
        ((retry_count++))
    fi
done

# Install Arduopilot Gazebo Plugin
# echo_green "Building ArduPilot Gazebo Plugin"
# cd $ws_dir/ThirdParty/ardupilot_gazebo
# if [[ -d "build" ]]; then
#     echo_green "Build directory exists. Proceeding with the build process."
# else
#     echo_green "Build directory does not exist. Creating the directory."
#     mkdir "build"
# fi
# cd build
# cmake ..
# make 
# sudo make install
# echo_green "ArduPilot Gazebo Plugin Installed"

# Build ORB_SLAM2
echo_green "Building ORB_SLAM2"
cd $ws_dir/ThirdParty/ORB_SLAM2_NOETIC

echo_green "Building Pangolin Dependency"
cd Pangolin
if [[ -d "build" ]]; then
    echo_green "Build directory exists. Proceeding with the build process."
else
    echo_green "Build directory does not exist. Creating the directory."
    mkdir "build"
fi
cd build
cmake ..
make -j
echo_green "Pangolin Dependency Installed"

## Build the ROS Workspace
echo_green "Building the ROS Workspace"
cd $ws_dir
catkin_make
## Add the ROS Workspace to the bashrc
echo "Setting up the ROS Workspace"
echo "source $ws_dir/devel/setup.bash" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=$ws_dir/ThirdParty/ardupilot_gazebo/models" >> ~/.bashrc
echo "export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$ws_dir/ThirdParty/ORB_SLAM2_NOETIC/Examples/ROS" >> ~/.bashrc

echo_green "Building ORB_SLAM2"
cd $ws_dir/ThirdParty/ORB_SLAM2_NOETIC
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:$ws_dir/ThirdParty/ORB_SLAM2_NOETIC/Examples/ROS
chmod +x build.sh
./build.sh
echo_green "ORB_SLAM2 Installed"
echo_green "Building ORB_SLAM2 ROS Examples"
chmod +x build_ros.sh
./build_ros.sh
echo_green "ORB_SLAM2 ROS Examples Installed"

echo "Done"

# source /workspaces/swarm-uav/devel/setup.bash
# export GAZEBO_MODEL_PATH=/workspaces/swarm-uav/ThirdParty/ardupilot_gazebo/models
# export ROS_PACKAGE_PATH=/workspaces/swarm-uav/src:/opt/ros/noetic/share:/workspaces/swarm-uav/ORB_SLAM2_NOETIC/Examples/ROS
