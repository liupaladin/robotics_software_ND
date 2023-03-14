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

3. The result for the project running is shown below. The robot is able to identify itself quickly enough.


