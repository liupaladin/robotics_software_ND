#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class RobotPlanner
{
    public:
        RobotPlanner();

        /// @brief This function calls the command_robot service and send the motion command to it
        void drive_robot(const float lin_x, const float ang_z);

        /// @brief This callback function continuously analyze the subscribed image data and 
        /// generate motion command based on path planning.
        void process_image_callback(const sensor_msgs::Image img);

    private:
        ros::ServiceClient client;
        ros::NodeHandle n_;
};