# include "robot_planner.h"

RobotPlanner::RobotPlanner() {
    // Initialize the client
    client = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
}

void RobotPlanner::drive_robot(const float lin_x, const float ang_z) {
    ROS_INFO_STREAM("Moving the robot towards the ball");

    // Set motion commands
    ball_chaser::DriveToTarget driver_srv;
    driver_srv.request.linear_x = lin_x;
    driver_srv.request.angular_z = ang_z;
    driver_srv.response.msg_feedback = "Requested robot move";

    // Call the command robot service to control the robot
    if (!client.call(driver_srv)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

void RobotPlanner::process_image_callback(const sensor_msgs::Image img) {
    drive_robot(0.1, 0.5);
}