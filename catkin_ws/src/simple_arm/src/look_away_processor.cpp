#include "ArmController.h"

int main(int argc, char** argv) {
    // Initialize the look_away node and create a handle to it
    ros::init(argc, argv, "look_away");
    ros::NodeHandle n;

    // Initialize the image based arm mover
    ArmController auto_arm_mover;

    ros::Subscriber sub1 = n.subscribe("/simple_arm/joint_states", 10, &ArmController::joint_states_callback, &auto_arm_mover);
    ros::Subscriber sub2 = n.subscribe("rgb_camera/image_raw", 10, &ArmController::look_away_callback, &auto_arm_mover);

    // Handle ROS communication events
    ros::spin();

    return 0;
}