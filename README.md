# trajectory_planner

### Prerequiste:
This pacakge depends on [OSQP](https://osqp.org/), [OSQP-Eigen](https://github.com/robotology/osqp-eigen) (C++/Eigen version of OSQP). To install:
```
# install OSQP
cd PATH/TO/YOUR/PREFERED/DIRECTORY
git clone --recursive https://github.com/osqp/osqp
cd osqp
mkdir build && cd build
cmake -G "Unix Makefiles" ..
cmake --build .
cmake --build . --target install

# install OSQP-Eigen
cd PATH/TO/YOUR/PREFERED/DIRECTORY
git clone https://github.com/robotology/osqp-eigen.git
cd osqp-eigen
mkdir build && cd build
cmake ..
make
make install
```

TODO:
1. Trajectory moving visualization
2. Include Piecewise Linear trajectory when fail happens
3. Paper review and code understanding
4. multiprocessing
