# Swarm Robotics
## Build
### ORB_SLAM2
Notes on how to build ORB_SLAM2 on Ubuntu 20.04
Avoid errors during compilation:
- Use FFMPEG 4
- Use the latest OpenCV 3 not 3.2.0
- Use Pangolin stable v5.0 instead of master branch (user for development)


## 1st Time Setup
Steps to setup the development environment:
Run the following vscode tasks
- Run import thirdparty repos task
- Run setup task
Now
- reload vscode window

#### Ardupilot parameter:
1. This parameter setup is done using bash script (`setup_ardupilot.sh`)
- /workspaces/swarm-uav/ThirdParty/ardupilot/Tools/autotest/pysim/vehicleinfo.py 
- /workspaces/swarm-uav/ThirdParty/ardupilot/Tools/autotest/default_params
2. You need to configure QGC
- Application Settings -> 
- Comm Links -> 
- Server Address: 0.0.0.0
- TCP port to 8100 for 1st drone and 8200 for 2nd drone and so on.

## Drone Description
Links:
- https://github.com/ethz-asl/rotors_simulator/tree/master 
- https://github.com/ethz-asl/rotors_simulator/wiki

## EGO-Planner
### Videos
1- main_ws/  
2- formation_ws/  
3- tracking_ws/  
4- interlaced_ws/  
### Notes on performance
- EGO-Planner is not able to create plan in case of wide obstacles, it is a local planner which only considers near obstacle avoidance.
### Parameters
- flight_type: 1 to receive goals from rostopic, 2 to use given waypoints in the launch file

## Simulation Packages
- iris_gazebo
- realsense_gazebo_plugin
