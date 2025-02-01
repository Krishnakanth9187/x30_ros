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

bool is_message_updated_ = false; ///< Flag to check if message has been updated

// Callback for handling message updates
void OnMessageUpdate(uint32_t code){
    if (code == 0x0906){
        is_message_updated_ = true;
    }
}

// Function to initialize robot and related components
void setup_robot(SendToRobot& send2robot_cmd, ParseCommand& robot_data_rec, MotionSDKMoveRobot& robot_set_up_demo) {
    robot_data_rec.RegisterCallBack(OnMessageUpdate);
    robot_data_rec.startWork();

    send2robot_cmd.robot_state_init();  // Initialize robot to default state

    RobotDataSDK* robot_data = &robot_data_rec.getRecvState();
    robot_set_up_demo.GetInitData(*robot_data, 0.000);  // Get initial joint states
}

// Main function to setup the node and run
int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);  // Initialize ROS2 context

    // Create shared pointers for components to manage memory
    auto send2robot_cmd = std::make_unique<SendToRobot>();
    auto robot_data_rec = std::make_unique<ParseCommand>();
    auto robot_set_up_demo = std::make_shared<MotionSDKMoveRobot>();

    // Setup robot and its initial states
    // setup_robot(*send2robot_cmd, *robot_data_rec, *robot_set_up_demo);

    // Create the ROS2 node for spinning and performing tasks
    // auto node = std::make_shared<rclcpp::Node>("robot_motion_node");

    // Start spinning the node to handle ROS2 callbacks
    rclcpp::spin(robot_set_up_demo);

    rclcpp::shutdown();  // Shutdown ROS2 context
    return 0;
}
