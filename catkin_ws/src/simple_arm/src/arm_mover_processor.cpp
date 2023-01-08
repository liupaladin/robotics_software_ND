#include "ArmActuator.h"

int main(int argc, char** argv) {
    // Initialize ros node
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle arm_mover_node_handle;

    // Initialize the arm mover actuator
    ArmActuator arm_actuator;

    // Define a safe_move service with a handle_safe_move_reqest callback function.
    ros::ServiceServer service = arm_mover_node_handle.advertiseService("/arm_mover/safe_move", &ArmActuator::handle_safe_move_request, &arm_actuator);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

}