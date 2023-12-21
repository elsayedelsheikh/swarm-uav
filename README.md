# Swarm Robotics
## Build
### ORB_SLAM2
Notes on how to build ORB_SLAM2 on Ubuntu 20.04
Avoid errors during compilation:
- Use FFMPEG 4
- Use the latest OpenCV 3 not 3.2.0
- Use Pangolin stable v5.0 instead of master branch (user for development)

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