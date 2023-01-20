#include "robot_planner.h"

int main(int argc, char** argv) {
    // Initialize the process image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Initialize the robot motion planner
    RobotPlanner robot_planner;
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, &RobotPlanner::process_image_callback, &robot_planner);

    // Handle ROS communication events
    ros::spin();

    return 0;
}