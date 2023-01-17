#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

class RobotPlanner
{
    public:
        RobotPlanner();

        /// @brief This callback function continuously analyze the subscribed image data and 
        /// generate motion command based on path planning.
        void process_image_callback(const sensor_msgs::Image& img);

    private:
        ros::ServiceClient client;
        ros::NodeHandle n_;

        /// @brief This function calls the command_robot service and send the motion command to it
        void drive_robot(const float lin_x, const float ang_z);

        /// @brief Find the list of white ball image segment center index
        /// @param img The image data from camera sensing
        /// @param target_pixel_value [0-255] The pixel color value to search for, assuming pure color object
        /// @return white_pixel_index, vector that stores all center indices of white ball per row
        std::vector<int> find_white_ball_center(const sensor_msgs::Image& img, const int target_pixel_value);
};