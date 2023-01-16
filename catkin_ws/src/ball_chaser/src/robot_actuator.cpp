#include "robot_actuator.h"

RobotActuator::RobotActuator() {
    // Define the publisher for wheel joint commands
    robot_command_pub_ = robot_mover_node_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
}

std::vector<float> RobotActuator::clamp_wheel_cmds_to_boundaries(const float linear_vel_cmd, const float angular_vel_cmd) {
    // Initialize the clamped commands
    float clamped_linear_vel_cmd = linear_vel_cmd;
    float clamped_angular_vel_cmd = angular_vel_cmd;

    // Get min and mx commands from config
    float min_linear_vel, max_linear_vel, min_angular_vel, max_angular_vel;

    // Initialize a node for ros config reading
    ros::NodeHandle node_config;
    std::string node_name = ros::this_node::getName();

    node_config.getParam(node_name + "/min_linear_vel", min_linear_vel);
    node_config.getParam(node_name + "/max_linear_vel", max_linear_vel);
    node_config.getParam(node_name + "/min_angular_vel", min_angular_vel);
    node_config.getParam(node_name + "/max_angular_vel", max_angular_vel);

    ROS_INFO("min_linear_vel j1 %1.2f", min_linear_vel);
    ROS_INFO("max_linear_vel j1 %1.2f", max_linear_vel);
    ROS_INFO("min_angular_vel j1 %1.2f", min_angular_vel);
    ROS_INFO("max_angular_vel j1 %1.2f", max_angular_vel);

    // Apply range protection for motor commands
    clamped_linear_vel_cmd = std::min(std::max(linear_vel_cmd, min_linear_vel), max_linear_vel);
    clamped_angular_vel_cmd = std::min(std::max(angular_vel_cmd, min_angular_vel), max_angular_vel);
    return {clamped_linear_vel_cmd, clamped_angular_vel_cmd};
}

bool RobotActuator::send_cmd_to_robot(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res) {
    ROS_INFO("DriveToTarget Request is received - j1 %1.2f, j2: %1.2f", (float)req.linear_x, (float)req.angular_z);
    // Check if the wheel commands are within range. Clamp them is range is violated
    std::vector<float> clamped_wheel_commands = clamp_wheel_cmds_to_boundaries((float)req.linear_x, (float)req.angular_z);
    // Set the wheel commands
    geometry_msgs::Twist wheel_command;
    wheel_command.linear.x = clamped_wheel_commands[0];
    wheel_command.angular.z = clamped_wheel_commands[1];
    robot_command_pub_.publish(wheel_command);

    // Wait 0.1 seconds for the wheel to settle
    ros::Duration(0.1).sleep();

    // Return a response message
    res.msg_feedback = "Wheel command set - linear_x at: " + std::to_string(wheel_command.linear.x) + ", angular_z at: " + std::to_string(wheel_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}