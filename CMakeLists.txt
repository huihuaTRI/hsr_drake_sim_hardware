cmake_minimum_required(VERSION 2.8.3)
project(hsr_drake_sim_hardware)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS
    actionlib
    actionlib_msgs
    controller_manager
    geometry_msgs
    hardware_interface
    pluginlib
    rosconsole
    roscpp
    tf2
    tf2_ros
    visualization_msgs
    lcm_to_ros
)

find_package(Eigen3 CONFIG REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  INCLUDE_DIRS include
#  LIBRARIES hsr_drake_sim_hardware
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  src
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

## Declare a C++ library
add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}/main.cc
)
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME hsr_drake_sim_hardware PREFIX "")
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  gflags
)
