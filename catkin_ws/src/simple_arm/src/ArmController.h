#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Image.h>

class ArmController
{
    public:
        ArmController();

        ///@brief This function calls the safe_move service define in arm_mover.cpp
        void move_arm_to_center();

        ///@brief update joint states based on sensor measurements
        void joint_states_callback(const sensor_msgs::JointState js);

        /// @brief Decide if to move the arm based on the image from camera
        void look_away_callback(const sensor_msgs::Image img);

    private:
        ros::ServiceClient client;
        ros::NodeHandle n_;
        bool moving_state_ = false;
        std::vector<double> joints_last_position_{0.0, 0.0};
};