This package is for project 2 and 3.
# For project 3
## Project run instruction
1. Go to the catkin_ws folder and use catkin_make to build all packages for the first time.
2. Open terminal and use `ctrl+shift+m` to open a second terminal and follow the following steps

    * Use source devel/setup.bash in both terminals.
    * Enter `roslaunch my_robot world.launch` to launch the gazebo world with the setup. The RVIZ should pop out with all the settings ready.
    * Load the map_server and AMCL algorithm using `roslaunch my_robot amcl.launch` in the second terminal.
    * Add a terminal using `ctrl+shift+m` and enter `rosrun teleop_twist_keyboard teleop_twist_keyboard.py` to localize the robot through manual driving. Recommended this approach since it's more dynamic than setting waypoint in rviz.
        * The package is also able to use to `2D Nav Goal` approach in RVIZ to localize the robot.

3. The results for the project running are shown below for two different end position. The robot is able to identify itself quickly enough.
<img width="1512" alt="Screen Shot 2023-03-13 at 1 54 45 PM" src="https://user-images.githubusercontent.com/51255025/224911191-a5408592-0525-4207-9684-4560aa44d35c.png">
<img width="1512" alt="Screen Shot 2023-03-13 at 10 42 18 PM" src="https://user-images.githubusercontent.com/51255025/224911438-aa6a37a3-5e35-4971-8c99-fa8faf2741c4.png">






