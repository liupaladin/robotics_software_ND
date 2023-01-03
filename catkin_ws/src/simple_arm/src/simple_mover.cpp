#include "ros/ros.h"
#include "std_msgs/Float64.h"

int main(int argc, char** argv){
    // Initialize ros node
    ros::init(argc, argv, "arm_mover");
    // Initialize node handle for the arm mover node
    ros::NodeHandle arm_mover_node_handle;
    // Create a publisher for the joint1 controller cmd
    ros::Publisher joint1_pub = arm_mover_node_handle.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    // Create a publish for the joint2 controller cmd
    ros::Publisher joint2_pub = arm_mover_node_handle.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Set loop frequency of 10 Hz
    ros::Rate loop_rate(10);

    int start_time, time_elapsed;

    // Get ROS start time
    while (not start_time) {
        start_time = ros::Time::now().toSec();
    }

    while (ros::ok()) {
        // Get elapsed time
        time_elapsed = ros::Time::now().toSec() - start_time;

        // Set the arm joint angles
        std_msgs::Float64 joint1_angle, joint2_angle;
        joint1_angle.data = sin(2 * M_PI * 0.1 * time_elapsed) * (M_PI / 2);
        joint2_angle.data = sin(2 * M_PI * 0.1 * time_elapsed) * (M_PI / 2);

        // Publish the arm joint angles
        joint1_pub.publish(joint1_angle);
        joint2_pub.publish(joint2_angle);

        // Sleep for the time remaining until 1/loop_rate is reached for the frame time
        loop_rate.sleep();
    }

    return 0;

}