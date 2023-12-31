# Trajectory Planning Library for Autonomous Robots 
This repo implements multiple trajectory optimization methods, such as min-snap trajectory planner, [ViGO](https://ieeexplore.ieee.org/abstract/document/10160638) (our local trajectory planner), based on the occupancy voxel map and the Octomap for autonomous robots.

**Author**: [Zhefan Xu](https://zhefanxu.com/), Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).

If you find this work helpful, kindly show your support by giving us a free ⭐️. Your recognition is truly valued.

This repo can be used as a standalone package and also comes as a module of our [autonomy framework](https://github.com/Zhefan-Xu/CERLAB-UAV-Autonomy).

## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [map_manager](https://github.com/Zhefan-Xu/map_manager) which provides the occupancy voxel map implementation and [octomap](http://wiki.ros.org/octomap) for octree-based map. It also depends on [global_planner](https://github.com/Zhefan-Xu/global_planner) for global waypoint generation.

```
# install dependency
sudo apt install ros-[melodic/noetic]-octomap* # octomap

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/trajectory_planner.git

cd ~/catkin_ws
catkin_make
```

## II. Run Planner DEMO
a. The example of the **local** trajectory planning using the B-spline-based trajectory optimization ([ViGO](https://ieeexplore.ieee.org/abstract/document/10160638)):

Please modify the correct path for the prebuilt map in ```cfg/bspline_interactive/occupancy_map.yaml```. All the prebuilt maps are under ```trajectory_planner/map```.

```
roslaunch trajectory_planner bspline_interactive.launch
```
Use ```2D Nav Goal``` in ```Rviz``` to select start and goal position in the map as shown below:

https://github.com/Zhefan-Xu/trajectory_planner/assets/55560905/dd5b4dc1-2290-44f5-b657-202f589ec772

The related paper can be found on:

**Zhefan Xu, Yumeng Xiu, Xiaoyang Zhan, Baihan Chen, and Kenji Shimada, “Vision-aided UAV Navigation and Dynamic Obstacle Avoidance using Gradient-based B-spline Trajectory Optimization”, IEEE International Conference on Robotics and Automation (ICRA), 2023.** [\[paper\]](https://ieeexplore.ieee.org/abstract/document/10160638) [\[video\]](https://youtu.be/xlMAL8aBHHg?si=4E5vShz7spxZDzps)

b. The example of the **global** trajectory planning using the min-snap trajectory optimization with the global planner:

```
roslaunch trajectory_planner poly_RRT_goal_interactive.launch 
```
Use ```2D Nav Goal``` in ```Rviz``` to select start and goal position in the map as shown below:

https://github.com/Zhefan-Xu/trajectory_planner/assets/55560905/08ac272e-6934-4ed8-a0b0-1fc4f4097dad

## III. Code Exmaple & API
a. For the B-spline-based trajectory optimization ([ViGO](https://ieeexplore.ieee.org/abstract/document/10160638)), the example code can be found in ```trajectory_planner/src/bspline_node.cpp```

b. For the min snap trajectory planner, the following code explains how to use the ```trajPlanner::polyTrajOctomap``` to generate collision-free trajectory from waypoints:
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
You can check ```src/poly_RRT_node.cpp``` for more details.

## IV. Parameters:
All planners parameters can be edited and modified in ```trajectory_planner/cfg/***.yaml```. 


## V. Issues
If you cannot visulize the Octomap with ROS noetic, that is because the [octomap_ros](http://wiki.ros.org/octomap) package has some tf name incompatibility issue. To solve that, please try building the octomap_ros from [source](https://github.com/OctoMap/octomap_mapping) with the following steps:

**Step1: download the octomap mapping source files**

```
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
```

**Step2: modify source files and catkin make**
Please change all the frame id ```/map``` to ```map``` in ```octomap_mapping/octomap_server/OctomapServer.cpp``` and ```octomap_mapping/octomap_server/octomap_server_static.cpp```. There should be 2 places. 
```
cd ~/catkin_ws
catkin_make
```

## VI. Citation and Reference
If you find this work useful, please cite the paper:
```
@inproceedings{xu2023vision,
  title={Vision-aided UAV navigation and dynamic obstacle avoidance using gradient-based B-spline trajectory optimization},
  author={Xu, Zhefan and Xiu, Yumeng and Zhan, Xiaoyang and Chen, Baihan and Shimada, Kenji},
  booktitle={2023 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={1214--1220},
  year={2023},
  organization={IEEE}
}
```
