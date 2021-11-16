cmake_minimum_required(VERSION 3.0.2)
project(my_local_planner)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  cmake_modules
  costmap_2d
  dynamic_reconfigure
  nav_core
  nav_msgs
  pluginlib
  sensor_msgs
  roscpp
  tf2
  tf2_geometry_msgs
  tf2_ros
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS 
    nav_msgs 
    roscpp 
    std_msgs 
    message_runtime
    move_base_msgs
    move_base
    visualization_msgs
    base_local_planner
    nav_core
    
  )

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_local_planner.cpp
# )

add_library(local_planner_lib src/local_planner.cpp)
target_link_libraries(local_planner_lib ${catkin_LIBRARIES})
