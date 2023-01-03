#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>
#include <algorithm>

// Global joint publisher variables
ros::Publisher joint1_pub, joint2_pub;

/// @brief This function checks and clamps the joint angles to a safe zone
/// @param requested_j1, [rad] the joint 1 command
/// @param requested_j2, [rad] the joint 2 command
std::vector<float> clamp_cmd_at_boundaries(float requested_j1, float requested_j2) {
    // Define the joint commands after range protection is applied
    float clamped_j1_cmd = requested_j1;
    float clamped_j2_cmd = requested_j2;

    // Get min and max joint parameters from config
    float min_j1, max_j1, min_j2, max_j2;
    
    // Initialize a node for ros config reading
    ros::NodeHandle node_config;
    std::string node_name = ros::this_node::getName();
    node_config.getParam(node_name + "/min_joint_1_angle", min_j1);
    node_config.getParam(node_name + "/max_joint_1_angle", max_j1);
    node_config.getParam(node_name + "/min_joint_2_angle", min_j2);
    node_config.getParam(node_name + "/max_joint_2_angle", max_j2);

    // Apply range protection for joint 1 and 2 if boundaries are violated
    clamped_j1_cmd = std::min(std::max(requested_j1, min_j1), max_j2);
    clamped_j2_cmd = std::min(std::max(requested_j2, min_j2), max_j2);
    std::vector<float> clamped_joint_cmds = {clamped_j1_cmd, clamped_j2_cmd};
    return clamped_joint_cmds;
}

bool handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res) {
    ROS_INFO("GoToPositionRequerst received - j1: %1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);
    // Check the joint command and apply range protection if necessary
    std::vector<float> clamped_joint_cmds = clamp_cmd_at_boundaries(req.joint_1, req.joint_2);
    // Set the arm joint angles
    std_msgs::Float64 joint1_angle, joint2_angle;
    joint1_angle.data = clamped_joint_cmds[0];
    joint2_angle.data = clamped_joint_cmds[1];
    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Joint angles set - j1 at: " + std::to_string(clamped_joint_cmds[0]) + ", j2 at: " + std::to_string(clamped_joint_cmds[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv) {
    // Initialize ros node
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle arm_mover_node_handle;

    // Define two publishers to publish joint controller commands
    joint1_pub = arm_mover_node_handle.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = arm_mover_node_handle.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Define a safe_move service with a handle_safe_move_reqest callback function.
    ros::ServiceServer service = arm_mover_node_handle.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

}