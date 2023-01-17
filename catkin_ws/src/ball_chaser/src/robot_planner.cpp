# include "robot_planner.h"

RobotPlanner::RobotPlanner() {
    // Initialize the client
    client = n_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");
}

void RobotPlanner::drive_robot(const float lin_x, const float ang_z) {
    ROS_INFO_STREAM("Moving the robot towards the ball");

    // Set motion commands
    ball_chaser::DriveToTarget driver_srv;
    driver_srv.request.linear_x = lin_x;
    driver_srv.request.angular_z = ang_z;
    driver_srv.response.msg_feedback = "Requested robot move";

    // Call the command robot service to control the robot
    if (!client.call(driver_srv)) {
        ROS_ERROR("Failed to call service command_robot");
    }
}

void RobotPlanner::process_image_callback(const sensor_msgs::Image img) {
    const int white_pixel = 255;
    int step_size = img.step;

    std::vector<int> white_pixel_index;
    for (int i_row = 0; i_row < img.height; i_row++) {
        int white_pixel_start_index = -1, white_pixel_end_index = -1;
        for (int j_pixel = 0; j_pixel < step_size; j_pixel++) {
            // Assume the white pixel region is continuous
            // Find the start index for white pixels
            bool is_white_pixel = j_pixel + 2 <step_size && img.data[i_row * step_size + j_pixel] == white_pixel && img.data[i_row * step_size + j_pixel + 1] == white_pixel && img.data[i_row * step_size + j_pixel + 2] == white_pixel;
            if (is_white_pixel && white_pixel_start_index == -1) {
                white_pixel_start_index = j_pixel;
            }
            // Find the end index for white pixels
            bool is_next_white_pixel = j_pixel + 5 <step_size && img.data[i_row * step_size + j_pixel + 3] == white_pixel && img.data[i_row * step_size + j_pixel + 4] == white_pixel && img.data[i_row * step_size + j_pixel + 5] == white_pixel;
            if (is_white_pixel && !is_next_white_pixel) {
                white_pixel_end_index = j_pixel;
                break;
            }
        }
        if (white_pixel_start_index != -1 && white_pixel_end_index != -1) {
                int white_pixel_center_index = white_pixel_start_index + (white_pixel_end_index - white_pixel_start_index) / 2;
                white_pixel_index.push_back(white_pixel_center_index);
        }
        // reset search indices for the next iteration
        white_pixel_start_index = -1;
        white_pixel_end_index = -1;
    }

    const float search_ang_z = 0.5; // Angular speed for searching rotation
    const float forward_speed = 0.1;
    const float k_p = 0.0002;

    // compute average center index per row to infer ball location
    int sum = 0;
    int average_index = 0;

    if (!white_pixel_index.empty()) {
        for (int index_value : white_pixel_index) {
            sum += index_value;
        }
        average_index = std::max(std::min(int(sum / white_pixel_index.size()), step_size), 0);
    } else {
        ROS_INFO("No ball is found in camera, enter self search mode");
        // Rotate robot to search for ball
        drive_robot(0.0, search_ang_z);
        return;
    }

    ROS_INFO("White ball index position is %1.4d", average_index);
    int delta_index = step_size / 2 - average_index;
    float turning_cmd = k_p * delta_index;
    ROS_INFO("Turning cmd is set at %1.3f", turning_cmd);
    if (average_index > 0 && average_index < step_size) {
        // Move towards left
        if (turning_cmd > 0) {
            ROS_INFO("Move towards left side");
        } else if (turning_cmd < 0) {
            ROS_INFO("Move towards right side");
        } else {
            ROS_INFO("Move straight");
        }
        drive_robot(forward_speed, turning_cmd);
    } else {
        // Don't move since no ball is identified
        drive_robot(0.0, 0.0);
    }
}