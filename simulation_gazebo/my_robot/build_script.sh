#!/usr/bin/bash
mkdir build && cd build
cmake ../
make
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/Users/jingxuanliu/workspace/robotics_software_ND/simulation_gazebo/my_robot/build