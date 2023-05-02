# Project 5: Home service robot
The strategy to detect the robot location status for updating marker is by adding a message topic to indicate if robot reaches each stage in the pick_objects node and subscribes in add_markers node.
## Project description
1. Go to the catkin_ws folder and use catkin_make to build all packages for the first time.
2. Open terminal and cd to `src/home_service_robot/scripts` and run home_service_robot.sh.
3. Watch the robot to do the job: The image below shows the results.
- First, as instructed the robot starts at the initial position with the virtual object appeared to be picked up.
![image](https://user-images.githubusercontent.com/51255025/235610389-6defdb87-c81e-4446-b859-92b940b3ac2a.png)
- Once the robot reaches the goal, the robot picks the object up and ready to move to delivery location.
![image](https://user-images.githubusercontent.com/51255025/235610836-f97547d3-e108-4284-9532-17a905e25daa.png)
- After picked up the object, the topic updates for the robot location status. The object disappears to emulate being carried.
![image](https://user-images.githubusercontent.com/51255025/235611115-7b4f3f33-483f-4968-9ef3-c263e2ed9452.png)
- Finally, when the object is delivered, the object emerged again.
![image](https://user-images.githubusercontent.com/51255025/235611301-394e24e4-31ee-4be4-9362-e6e8e461c122.png)
