#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>
#include <algorithm>

class ArmActuator {

    public:
        ArmActuator();

        /// @brief This function checks and clamps the joint angles to a safe zone
        /// @param requested_j1, [rad] the joint 1 command
        /// @param requested_j2, [rad] the joint 2 command
        std::vector<float> clamp_cmd_at_boundaries(float requested_j1, float requested_j2);

        /// @brief This function sends the joint commands to the actuator to move the arm
        bool handle_safe_move_request(simple_arm::GoToPosition::Request& req, simple_arm::GoToPosition::Response& res);

    private:
        ros::NodeHandle arm_actuator_node_;
        // Publisher for joint position commands
        ros::Publisher joint1_pub, joint2_pub;

};