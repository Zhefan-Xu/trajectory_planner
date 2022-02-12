# Trajectory Planner for Quadcopter 
This repo implements minimum snap trajectory optimization methods for quadcopters based on [Octomap](http://wiki.ros.org/octomap) (C++/ROS). 

## Get Started:
This packages allow you generating collision-free trajectory based on your map. You can set start and goal by clicking or use code API.
- #### Quick DEMO:
   
   https://user-images.githubusercontent.com/55560905/153730534-7900340a-a9f6-4301-ab42-569a87c0c1a3.mp4

## Installation
- #### Prerequiste:
    This pacakge depends on [OSQP](https://osqp.org/), [OSQP-Eigen](https://github.com/robotology/osqp-eigen) (C++/Eigen version of OSQP), [Octomap](http://wiki.ros.org/octomap). Please go to their official website for installation. If you are not familiar with CMake, you can follow the instruction below:
    ```
    # install OSQP
    cd PATH/TO/YOUR/PREFERED/DIRECTORY
    git clone --recursive https://github.com/osqp/osqp
    cd osqp
    mkdir build && cd build
    cmake -G "Unix Makefiles" ..
    cmake --build .
    sudo cmake --build . --target install

    # install OSQP-Eigen
    cd PATH/TO/YOUR/PREFERED/DIRECTORY
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    mkdir build && cd build
    cmake ..
    make
    sudo make install

    # install octoamp related packages
    sudo apt install ros-noetic-octomap*
    ```
- #### install:
    To realize global navigation, RRT/RRT* planner is used the global planner in this [repo](https://github.com/Zhefan-Xu/global_planner). It is not required if you want to write your own code using this repo. It is just for navigation demo.
    ```
    cd ~/catkin_ws/src
    git clone https://github.com/Zhefan-Xu/global_planner.git # OPTIONAL (ONLY required if you want to play with our demo)
    git clone https://github.com/Zhefan-Xu/trajectory_planner.git

    # make
    cd ~/catkin_ws
    catkin_make
    ```

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



