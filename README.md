# Trajectory Planner for Quadcopter 
This repo implements minimum snap trajectory optimization methods for quadcopters based on [Octomap](http://wiki.ros.org/octomap) (C++/ROS). 

## Quick DEMO:
This packages allow you generating collision-free trajectory based on your map. You can set start and goal by clicking or use code API.

https://user-images.githubusercontent.com/55560905/153730534-7900340a-a9f6-4301-ab42-569a87c0c1a3.mp4

## Installation
#### Prerequiste:
This pacakge depends on [OSQP](https://osqp.org/), [OSQP-Eigen](https://github.com/robotology/osqp-eigen) (C++/Eigen version of OSQP). Please go to their official website for installation. If you are not familiar with CMake, you can follow the instruction below:
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
```
#### install:
To realize global navigation, RRT/RRT* planner is used the global planner in this [repo](https://github.com/Zhefan-Xu/global_planner). It is not required if you want to write your own code using this repo. It is just for navigation demo.
```
git clone https://github.com/Zhefan-Xu/global_planner.git # OPTIONAL (ONLY required if you want to play with our demo)

```







https://user-images.githubusercontent.com/55560905/153730537-33c7e471-3f19-4082-95e6-3fac7c35a09a.mp4

https://user-images.githubusercontent.com/55560905/153730536-22a3fd27-dd7c-4cc1-b23d-0d629490a435.mp4





