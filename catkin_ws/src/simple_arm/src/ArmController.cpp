#include "ArmController.h"

ArmController::ArmController() {
    // Initialize the client
    client = n_.serviceClient<simple_arm::GoToPosition>("/arm_mover/safe_move");
}

void ArmController::move_arm_to_center() {
    ROS_INFO_STREAM("Moving the arm to the center");

    // Request centered joint angles 
    simple_arm::GoToPosition srv;
    srv.request.joint_1 = 1.57;
    srv.request.joint_2 = 1.57;

    // Call the safe_move service and passed the joint angles
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service safe_move");
    }
}

void ArmController::joint_states_callback(const sensor_msgs::JointState js) {
    // Get the joints current position from the topic subscribed
    std::vector<double> joints_current_position = js.position;

    // Define a tolerance threshold to compare double values
    double tolerance = 0.0005;

    // Check if the arm is moving by checking difference between previous and current joints position
    if (fabs(joints_current_position[0] - joints_last_position_[0]) < tolerance && fabs(joints_current_position[1] - joints_last_position_[1]) < tolerance) {
        moving_state_ = false;
    } else {
        moving_state_ = true;
    }
    joints_last_position_ = joints_current_position;
}

void ArmController::look_away_callback(const sensor_msgs::Image img) {
    bool uniform_image = true;

    //loop through all pixels in the image and check if they are all identical
    for (int i = 0; i < img.height * img.step - 1; i++) {
        if (img.data[i + 1] - img.data[i] != 0) {
            uniform_image = false;
            break;
        }
    }

    if (uniform_image == true && moving_state_ == false) {
        move_arm_to_center();
    }
}
