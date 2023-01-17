# Project 2: Go chase it
This document serves as the project summary for reviewers reference. 

## Summary

In this project, I finished the following major tasks as required:

- Wrote drive_bot and process_image node and tested the functionality of both nodes. I also went further to use the recommended class based implementation for creating service and clients.

    - robot_actuator class is in charge of the actual service and send command to move the robot.
    - robot_planner class is in charge of image processing and generate commands to be sent by service. The controller algorithm applied is a portional based controller using ball position to center of image as the feedback error.
- The world is created and a white ball is put into the scene. The robot is created with the sensors installed. All sensors reads correctly.

- **Bonus**: instead of having the robot stay still when not seeing any white ball, I added self seach capability so that it can search by itself for ball in the room by rotating. This works well and makes the chase game continous and adds more intelligence to the robot.