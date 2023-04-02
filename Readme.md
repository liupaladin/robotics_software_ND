This repository is used for robotics software nano degree program from udacity
# Project 4: Mapping the world
## Project description
1. Go to the catkin_ws folder and use catkin_make to build all packages for the first time.
2. Open terminal and use `ctrl+shift+m` to open two more terminal and follow the following steps

    * Use source devel/setup.bash in each terminals under catkin_ws folder
    * Enter `roslaunch my_robot world.launch` to launch the gazebo world with the setup. The RVIZ should pop out with all the settings ready.
    * Enter `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to move the robot through manual driving. Move it slow along the wall of the room helps a quick mapping convergence
    * Load the mapping node using `roslaunch my_robot mapping.launch` in the second terminal.
    * The map will be saved once mapping launch is terminated in process.
3. Project result
- The view when all the windows are loaded for mapping is shown below.

- After the mapping process is completed, you can open the map at ~/.ros location by default and use `rtabmap-databaseViewer ~/.ros/rtabmap.db`. For review convenience, the db file is copied to /myrobot/maps folder. The image below shows the outcome of the mapping process. The map is well identified and the occupancy grid is clear to be used.

# Project 3: where am I
## Project run instruction
1. Go to the catkin_ws folder and use catkin_make to build all packages for the first time.
2. Open terminal and use `ctrl+shift+m` to open a second terminal and follow the following steps

    * Use source devel/setup.bash in both terminals.
    * Enter `roslaunch my_robot world.launch` to launch the gazebo world with the setup. The RVIZ should pop out with all the settings ready.
    * Load the map_server and AMCL algorithm using `roslaunch my_robot amcl.launch` in the second terminal.
    * Add a terminal using `ctrl+shift+m` and enter `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to localize the robot through manual driving. Recommended this approach since it's more dynamic than setting waypoint in rviz.
        * The package is also able to use to `2D Nav Goal` approach in RVIZ to localize the robot.

3. The results for the project running are shown below for two different end position. The robot is able to identify itself quickly enough.
<img width="1512" alt="Screen Shot 2023-03-14 at 7 53 55 PM" src="https://user-images.githubusercontent.com/51255025/225193884-5779e98b-a9f2-4ea6-a541-d59ac0a57d8c.png">
<img width="1512" alt="Screen Shot 2023-03-14 at 7 54 37 PM" src="https://user-images.githubusercontent.com/51255025/225193788-7dd24073-26db-46e6-89e8-9573e7ebc94e.png">

# Project 1:
## Introduction
This folder is for project one. The space created is a typical single floor type plan. I put two four wheel robots and one three wheel robots inside. 

## Instruction
- To build the script for plugin, please follow the following steps:
  - `mkdir build && cd build`
  - `cmake ../`
  - `make`
  - `export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/Users/jingxuanliu/workspace/robotics_software_ND/simulation_gazebo/project_1_build_my_world/build`

- To load the gazebo world for project 1, navigate to simulation_gazebo/project_1_build_my_world/world and then use command `gazebo project_1_single_floor_env` in any terminal.

- The model files are stored inside the model folder. The building is also stored in model folder as instructerd with name "robot_house_space".


