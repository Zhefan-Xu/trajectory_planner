cmake_minimum_required(VERSION 3.0.2)
project(trajectory_planner)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "-std=c++14 ${CMAKE_CXX_FLAGS} -O3 -Wall")
add_compile_options(-w)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  octomap_ros
  global_planner
  map_manager
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(OsqpEigen REQUIRED)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES trajectory_planner
 CATKIN_DEPENDS roscpp rospy std_msgs 
 # DEPENDS OsqpEigen
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}/third_party/ACADO/acado
  include/${PROJECT_NAME}/third_party/ACADO
  include/${PROJECT_NAME}/mpc_solver
  include/${PROJECT_NAME}/mpc_solver/qpoases
  include/${PROJECT_NAME}/mpc_solver/qpoases/INCLUDE 
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC
  # ${OsqpEigen_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory_planner.cpp
# )
# add_library(polyTrajSolver include/${PROJECT_NAME}/polyTrajSolver.cpp)
# add_library(polyTrajOctomap include/${PROJECT_NAME}/polyTrajOctomap.cpp)
# add_library(pwlTraj include/${PROJECT_NAME}/piecewiseLinearTraj.cpp)
add_library(
	${PROJECT_NAME} 
  include/${PROJECT_NAME}/path_search/astarOcc.cpp
	include/${PROJECT_NAME}/polyTrajSolver.cpp 
	include/${PROJECT_NAME}/polyTrajOctomap.cpp
	include/${PROJECT_NAME}/piecewiseLinearTraj.cpp
  include/${PROJECT_NAME}/bspline.cpp
  include/${PROJECT_NAME}/bsplineTraj.cpp
  include/${PROJECT_NAME}/polyTrajOccMap.cpp
  include/${PROJECT_NAME}/clustering/Kmeans.cpp
  include/${PROJECT_NAME}/clustering/obstacleClustering.cpp
  include/${PROJECT_NAME}/mpcPlanner.cpp
  include/${PROJECT_NAME}/obstacle_test/dynamicObstacleGenerator.cpp

  include/${PROJECT_NAME}/mpc_solver/acado_auxiliary_functions.c
  include/${PROJECT_NAME}/mpc_solver/acado_integrator.c
  include/${PROJECT_NAME}/mpc_solver/acado_qpoases_interface.cpp
  include/${PROJECT_NAME}/mpc_solver/acado_solver_sfunction.c
  include/${PROJECT_NAME}/mpc_solver/acado_solver.c
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/Bounds.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/Constraints.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/CyclingManager.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/Indexlist.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/MessageHandling.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/QProblem.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/QProblemB.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/SubjectTo.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/Utils.cpp
  include/${PROJECT_NAME}/mpc_solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/trajectory_planner_node.cpp)
add_executable(poly_RRT_node src/poly_RRT_node.cpp)
add_executable(poly_RRTStar_node src/poly_RRTStar_node.cpp)
add_executable(poly_RRT_goal_node src/poly_RRT_goal_node.cpp)
add_executable(poly_RRTStar_goal_node src/poly_RRTStar_goal_node.cpp)
add_executable(bspline_node src/bspline_node.cpp)
# add_executable(testEigen src/test/testEigen.cpp)
# add_executable(testTrajSolver src/test/testTrajSolver.cpp)
# add_executable(testBspline src/test/testBspline.cpp)
# add_executable(testBsplineFit src/test/testBsplineFit.cpp)
add_executable(testObstacleClustering src/test/testObstacleClustering.cpp)
add_executable(mpc_node src/mpc_node.cpp)
add_executable(mpc_solver_setup src/mpc_solver_setup.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
# target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} "${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_planner/third_party/libosqp.a" "${CMAKE_CURRENT_SOURCE_DIR}/include/trajectory_planner/third_party/libOsqpEigen.so.0.7.0")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  message("Detected x86_64 architecture.")
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/x86/libosqp.so ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/x86/libOsqpEigen.so.0.7.0 ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/ACADO/libacado_toolkit_s.so)
  # target_link_libraries(test_time_optimizer ${PROJECT_NAME})
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "arm")
  message("Detected ARM architecture.")
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libosqp.so ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libOsqpEigen.so.0.8.0 ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/ACADO/libacado_toolkit_s.so)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  message("Detected ARM64 architecture.")
  target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libosqp.so ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/lib/ARM/libOsqpEigen.so.0.8.0 ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/ACADO/libacado_toolkit_s.so)
else()
    message("Unknown architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()
# target_link_libraries(polyTrajSolver OsqpEigen::OsqpEigen)
# target_link_libraries(polyTrajOctomap polyTrajSolver pwlTraj)
target_link_libraries(poly_RRT_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(poly_RRTStar_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(poly_RRT_goal_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(poly_RRTStar_goal_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(bspline_node ${catkin_LIBRARIES} ${PROJECT_NAME})
# target_link_libraries(testEigen ${catkin_LIBRARIES} OsqpEigen::OsqpEigen)
# target_link_libraries(testTrajSolver ${catkin_LIBRARIES} ${PROJECT_NAME})
# target_link_libraries(testBspline ${catkin_LIBRARIES} ${PROJECT_NAME})
# target_link_libraries(testBsplineFit ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(testObstacleClustering ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(mpc_node ${catkin_LIBRARIES} ${PROJECT_NAME})
target_link_libraries(mpc_solver_setup ${PROJECT_SOURCE_DIR}/include/${PROJECT_NAME}/third_party/ACADO/libacado_toolkit_s.so)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION} 
  # FILES_MATCHING PATTERN "*.h"
  # PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory_planner.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
