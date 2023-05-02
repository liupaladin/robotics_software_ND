#!/bin/sh
xterm -e " roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=$(rospack find gmapping)/../../world/simple_room_service_proj.world" &
sleep 5
xterm -e " roslaunch turtlebot_gazebo amcl_demo.launch" &
sleep 5
xterm -e " roslaunch turtlebot_rviz_launchers view_navigation.launch" &
sleep 5
xterm -e " rosrun add_markers add_markers_test" &
sleep infinity