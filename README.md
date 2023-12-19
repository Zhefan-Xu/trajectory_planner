# Trajectory Planning Library for Autonomous Robots 
This repo implements multiple trajectory optimization methods, such as min-snap trajectory planner, [ViGO](https://ieeexplore.ieee.org/abstract/document/10160638) (our local trajectory planner), based on the occupancy voxel map and the Octomap for autonomous robots.

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [map_manager](https://github.com/Zhefan-Xu/map_manager) which provides the occupancy voxel map implementation and [octomap_ros](http://wiki.ros.org/octomap) for octree-based map. It also depends on [global_planner](https://github.com/Zhefan-Xu/global_planner) for global waypoint generation.

```
# install dependency
sudo apt install ros-[melodic/noetic]-octomap* # octomap

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/map_manager.git # occupancy voxel map. please refer to the original repo if you meet any issue.
git clone https://github.com/Zhefan-Xu/global_planner.git
git clone https://github.com/Zhefan-Xu/trajectory_planner.git

cd ~/catkin_ws
catkin_make
```

## II. Run Planner DEMO
a. Example of **local** trajectory planning using the B-spline-based trajectory optimization ([ViGO](https://ieeexplore.ieee.org/abstract/document/10160638)):

a. Example of **global** trajectory planning using the min-snap trajectory optimization with the global planner:

https://user-images.githubusercontent.com/55560905/153730534-7900340a-a9f6-4301-ab42-569a87c0c1a3.mp4



## Usage:
- #### I. Generate trajectory from start and goal click point:
    ```
    roslaunch trajectory_planner polyRRTInteractive.launch
    ```
    See quick demo video for its example.

- #### II. Navigation:
    ```
    roslaunch trajectory_planner polyRRTGoalInteractive.launch
    ```
    - Example 1 (Navigation):

    https://user-images.githubusercontent.com/55560905/153731254-ba3a311e-9c64-4056-8285-c466f475d7b1.mp4


    - Example 2 (Navigation):

    https://user-images.githubusercontent.com/55560905/153731258-8d41a6e3-d908-4fa0-9397-53baae93dd13.mp4

## Code API:
The following code explains how to use the ```trajPlanner::polyTrajOctomap``` to generate collision-free trajectory from waypoints. 
```
#include <trajectory_planner/polyTrajOctomap.h>

int main(){
    ...
    ros::NodeHandle nh;

    // Initialize Planner
    trajPlanner::polyTrajOctomap polyPlanner (nh); // trajectory planner

    // Load Waypoint Path and generate trajectory
    polyPlanner.updatePath(path); // path: nav_msgs::Path
    polyPlanner.makePlan();

    // Get pose from trajectory
    polyPlanner.getPose(t); // get pose at time t
    ...
}
```
You can check ```src/polyRRTNode.cpp``` for more details.

## Parameters:
Planner paramters can be edited in ```cfg/planner.yaml```. The paramter names are pretty self-explained.


## Reference:
- Mellinger, D. and Kumar, V., 2011, May. Minimum snap trajectory generation and control for quadrotors. In 2011 IEEE international conference on robotics and automation (pp. 2520-2525).
- Richter, C., Bry, A. and Roy, N., 2016. Polynomial trajectory planning for aggressive quadrotor flight in dense indoor environments. In Robotics research (pp. 649-666). Springer, Cham.


debug pwl
