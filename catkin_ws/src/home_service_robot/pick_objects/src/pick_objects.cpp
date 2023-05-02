#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/UInt8.h>
 
// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Setup publisher for robot location status
  ros::Publisher robot_loc_pub = n.advertise<std_msgs::UInt8>("/robot_loc_status", 1);
  std_msgs::UInt8 status_msg;
  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  
  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -2.5;
  goal.target_pose.pose.position.y = 3.5;
  goal.target_pose.pose.orientation.w = 1.0;

  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  
  // Wait an infinite time for the results
  ac.waitForResult();
  
  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot moved to the pickup position");
    ros::Duration(5.0).sleep();
    // Change robot location status to 1 for reached pickup location
    status_msg.data = 1;
    robot_loc_pub.publish(status_msg);
  } else {
    ROS_INFO("The robot failed to navigate to pickup position");
    return 0;
  }

  // Set second goal
  goal.target_pose.pose.position.x = -7.5;
  goal.target_pose.pose.position.y = -3.7;
  goal.target_pose.pose.orientation.w = 1.0;
  // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);
  // Update robot location status to 2 for moving to delivery location
  status_msg.data = 2;
  robot_loc_pub.publish(status_msg);
  // Wait an infinite time for the results
  ac.waitForResult();

  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_INFO("Hooray, the robot moved to the delivery position");
    // Change robot location status to 3 for reached delivery location
    status_msg.data = 3;
    robot_loc_pub.publish(status_msg);
  } else {
    ROS_INFO("Oops, the robot didn't manage to move to the delivery position");
    return 0;
  }
  ROS_INFO("Mission accomplished");
  ros::Duration(5.0).sleep();  
  return 0;
}