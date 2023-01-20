#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <geometry_msgs/Twist.h>
#include <vector>
#include <algorithm>

class RobotActuator {
    
    public:
        RobotActuator();

        /// @brief This function sends the wheel joint comment to the device through publisher
        bool send_cmd_to_robot(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res);

    private:
        /// @brief This function check and clamps the wheel joint command to feasible range
        /// @param linear_vel_cmd The speed command sent to wheel joint
        /// @param angular_vel_vmd The angular speed comment sent to wheel joint
        std::vector<float> clamp_wheel_cmds_to_boundaries(const float linear_vel_cmd, const float angular_vel_cmd);
        
        ros::NodeHandle robot_mover_node_;
        ros::Publisher robot_command_pub_;
};