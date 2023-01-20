#include "robot_actuator.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "drive_bot");
    ros::NodeHandle drive_bot_node_handle;

    // Initialize the robot actuator
    RobotActuator robot_actuator;

    // Define a robot motion actuator service with send_cmd_to_robot callback function.
    ros::ServiceServer service = drive_bot_node_handle.advertiseService("/ball_chaser/command_robot", &RobotActuator::send_cmd_to_robot, &robot_actuator);
    ROS_INFO("Read to send moving commands to robot");
    // Handle ROS communication events
    ros::spin();
}