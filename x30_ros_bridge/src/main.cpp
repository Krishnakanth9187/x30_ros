#include "rclcpp/rclcpp.hpp"
#include "x30_ros_bridge/udp_socket.hpp"
#include "x30_ros_bridge/udp_server.hpp"
#include "x30_ros_bridge/command_list.h"
#include "x30_ros_bridge/parse_cmd.h"
#include "x30_ros_bridge/send_to_robot.h"
#include "x30_ros_bridge/motion_sdk_move_robot.h"

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <iostream>
#include <time.h>
#include <string.h>

using namespace std;

bool is_message_updated_ = false; 



int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);  


    auto robot_set_up_demo = std::make_shared<MotionSDKMoveRobot>();

    rclcpp::spin(robot_set_up_demo);

    rclcpp::shutdown(); 
    return 0;
}
