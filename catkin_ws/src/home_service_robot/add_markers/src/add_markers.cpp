#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/UInt8.h>

uint8_t robot_localization_status = 0;

void robotLocStatusCallback(const std_msgs::UInt8::ConstPtr& msg) {
    robot_localization_status = msg->data;
    return;
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber robot_pub = n.subscribe("/robot_loc_status", 1, robotLocStatusCallback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  // Flag for detect if marker status
  bool is_marker_created = false;
  bool is_marker_disappeared = false;
  bool is_marker_recreated = false;

  while (ros::ok())
  {
    ros::spinOnce();
    ROS_INFO("Subscribed robot loc status is %d", robot_localization_status);
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    
    // Implement the marker generation sequence
    if (robot_localization_status == 0) {
        marker_pub.publish(marker);
        ROS_INFO("Create marker at pick up location");
        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = -2.5;
        marker.pose.position.y = 3.5;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker_pub.publish(marker);
        is_marker_created = true;
    } else if (robot_localization_status == 1) {
        sleep(5);
        ROS_INFO("Hiding marker after");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
    } else if (robot_localization_status == 2) {
        ROS_INFO("Waiting marker to be carried to destination");
        marker.action = visualization_msgs::Marker::DELETE;
        marker_pub.publish(marker);
    } else if (robot_localization_status == 3) {
        ROS_INFO("Add marker back at the delivery location");
        marker.pose.position.x = -7.5;
        marker.pose.position.y = -3.7;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.action = visualization_msgs::Marker::ADD;
        marker_pub.publish(marker);
        is_marker_recreated = true;
        is_marker_disappeared = false;
    } 
    r.sleep();
  }
}