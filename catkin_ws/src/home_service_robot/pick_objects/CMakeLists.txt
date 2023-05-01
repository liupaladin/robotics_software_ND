cmake_minimum_required(VERSION 2.8.3)
project(pick_objects)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED)

## System dependencies are found with CMake's conventions
find_package(catkin REQUIRED COMPONENTS 
  actionlib 
  move_base_msgs 
  roscpp
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(pick_objects src/pick_objects.cpp)

target_link_libraries(pick_objects ${catkin_LIBRARIES})
  