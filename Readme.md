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
![image](https://user-images.githubusercontent.com/51255025/229369216-2fe89338-ca5b-4b94-930e-4bee8dc7d21d.png)

- After the mapping process is completed, you can open the map at ~/.ros location by default and use `rtabmap-databaseViewer ~/.ros/rtabmap.db`. For review convenience, the db file is copied to /myrobot/maps folder. The image below shows the outcome of the mapping process. The map is well identified and the occupancy grid is clear to be used. The view below shows a point where loop closure is detected for a wall corner. The yellow and pink circles are expected based on the view. The loop closure in this process has 12 in total, meeting the project requirement.
![image2](https://user-images.githubusercontent.com/51255025/229369236-78a0466e-32ec-4b4a-a13a-0af7ba4ed52b.png)


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

# Project 2: Go chase it
This document serves as the project summary for reviewers reference. 

## Summary

In this project, I finished the following major tasks as required:

- Wrote drive_bot and process_image node and tested the functionality of both nodes. I also went further to use the recommended class based implementation for creating service and clients.

    - robot_actuator class is in charge of the actual service and send command to move the robot.
    - robot_planner class is in charge of image processing and generate commands to be sent by service. The controller algorithm applied is a portional based controller using ball position to center of image as the feedback error.
- The world is created and a white ball is put into the scene. The robot is created with the sensors installed. All sensors reads correctly.
    - Robot config is changed from the sample per requirements.

- **Bonus**: instead of having the robot stay still when not seeing any white ball, I added self search capability so that it can search by itself for ball in the room by rotating. This works well and makes the chase game continuous and adds more intelligence to the robot.

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


