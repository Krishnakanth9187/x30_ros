/**
 * @file motionexample.cpp
 * @author ysc (www.deeprobotics.cn)
 * @brief 
 * @version 0.1
 * @date 2023-03-03
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#include "x30_ros_bridge/motion_sdk_move_robot.h"
// Vec3 goal_angle_fl, goal_angle_hl, goal_angle_fr, goal_angle_hr;
double init_angle_fl[3], init_angle_fr[3], init_angle_hl[3], init_angle_hr[3];
double init_time;

double  kp_hipx=300;
double  kd_hipx=4;

double  kp_hipy=300;
double  kd_hipy=4;

double  kp_knee=300;
double  kd_knee=4;


MotionSDKMoveRobot::MotionSDKMoveRobot() : rclcpp::Node("motion_sdk_move_robot") {
    joint_trajectory_subscriber_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/joint_group_effort_controller/joint_trajectory", 10,
        std::bind(&MotionSDKMoveRobot::jointTrajectoryCallback, this, std::placeholders::_1));

        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        robot_data_rec = new ParseCommand;

        robot_data_rec->RegisterCallBack(std::bind(&MotionSDKMoveRobot::OnMessageUpdate, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), 
                                         std::bind(&MotionSDKMoveRobot::publishJointStates, this));

        RCLCPP_INFO(this->get_logger(), "MotionSDKMoveRobot Node Initialized");
}

void MotionSDKMoveRobot::jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Received JointTrajectory message with %zu points", msg->points.size());
    
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received an empty JointTrajectory message");
        return;
    }
    
    const auto& positions = msg->points[0].positions;
    if (positions.size() != 12) {
        RCLCPP_ERROR(this->get_logger(), "Expected 12 joint positions but received %zu", positions.size());
        return;
    }
    
    RobotCmdSDK cmd;

    double final_angles[3];
    double total_time = 1.0; 
    double run_time = 0.0;
    double cycle_time = 0.01; 

    std::vector<std::string> leg_sides = {"FL", "FR", "HL", "HR"};

    double* init_angles[4] = {init_angle_fl, init_angle_fr, init_angle_hl, init_angle_hr};

    RobotDataSDK& data = robot_data_rec->getRecvState();

    for (size_t i = 0; i < leg_sides.size(); i++) {
        final_angles[0] = positions[i * 3];
        final_angles[1] = positions[i * 3 + 1];
        final_angles[2] = positions[i * 3 + 2];

        SwingToAngle(init_angles[i], final_angles, total_time, run_time, cycle_time, leg_sides[i], cmd, data);
    }
}


 /**
  * @brief Only the current moment and angle are recorded
  * @param data Current joint data
  * @param time Current timestamp
  */
void MotionSDKMoveRobot::OnMessageUpdate(uint32_t code) {
        if (code == 0x0906) {
            is_message_updated_ = true;
        }
    }

// Publish joint states if new data is available
void MotionSDKMoveRobot::publishJointStates() {
    if (!is_message_updated_) return;

    RobotDataSDK* robot_data = &robot_data_rec->getRecvState();

      init_angle_fl[0] = robot_data->fl_leg[0].pos;
      init_angle_fl[1] = robot_data->fl_leg[1].pos;
      init_angle_fl[2] = robot_data->fl_leg[2].pos;

      init_angle_fr[0] = robot_data->fr_leg[0].pos;
      init_angle_fr[1] = robot_data->fr_leg[1].pos;
      init_angle_fr[2] = robot_data->fr_leg[2].pos;

      init_angle_hl[0] = robot_data->hl_leg[0].pos;
      init_angle_hl[1] = robot_data->hl_leg[1].pos;
      init_angle_hl[2] = robot_data->hl_leg[2].pos;

      init_angle_hr[0] = robot_data->hr_leg[0].pos;
      init_angle_hr[1] = robot_data->hr_leg[1].pos;
      init_angle_hr[2] = robot_data->hr_leg[2].pos;

    sensor_msgs::msg::JointState joint_state_msg;
    joint_state_msg.header.stamp = this->get_clock()->now();

    joint_state_msg.name = {"fl_leg_1", "fl_leg_2", "fl_leg_3",
                            "fr_leg_1", "fr_leg_2", "fr_leg_3",
                            "hl_leg_1", "hl_leg_2", "hl_leg_3",
                            "hr_leg_1", "hr_leg_2", "hr_leg_3"};

    joint_state_msg.position = {robot_data->fl_leg[0].pos, robot_data->fl_leg[1].pos, robot_data->fl_leg[2].pos,
                                robot_data->fr_leg[0].pos, robot_data->fr_leg[1].pos, robot_data->fr_leg[2].pos,
                                robot_data->hl_leg[0].pos, robot_data->hl_leg[1].pos, robot_data->hl_leg[2].pos,
                                robot_data->hr_leg[0].pos, robot_data->hr_leg[1].pos, robot_data->hr_leg[2].pos};


    // Publish joint state message
    joint_state_publisher_->publish(joint_state_msg);

    // Reset flag
    is_message_updated_ = false;
}


/**
 * @brief Specifically achieve swinging one leg of the robot to a specified position within a specified time
 * @param initial_angle 
 * @param final_angle
 * @param total_time
 * @param run_time
 * @param cycle_time Control cycle, default is 1ms
 * @param side Control which leg, FL is the left front leg, FR is the right front leg, HL is the left and right leg, and HR is the right rear leg
 * @param cmd Issue control command
 * @param data Real-time status data of robot
 */
void MotionSDKMoveRobot::SwingToAngle(double initial_angle[3], double final_angle[3],
                                 double total_time, double run_time,
                                 double cycle_time, string side,
                                 RobotCmdSDK &cmd,
                                 RobotDataSDK &data) {
  double goal_angle[3]={0};
  double goal_angle_next[3]={0};
  double goal_angle_next2[3]={0};
  double goal_vel[3]={0};
  int leg_side;

  if (side == "FL")
    leg_side = 0;
  else if (side == "FR")
    leg_side = 1;
  else if (side == "HL")
    leg_side = 2;
  else if (side == "HR")
    leg_side = 3;
  else
    cout << "Leg Side Error!!!" << endl;
   
  for (int j = 0; j < 3; j++) {
    CubicSpline(initial_angle[j], 0, (double)final_angle[j], 0, run_time,
                cycle_time, total_time, goal_angle[j], goal_angle_next[j],
                goal_angle_next2[j]);
    goal_vel[j] = (goal_angle_next[j] - goal_angle[j]) / cycle_time;       
  }


  

  // The following two methods can be used to complete joint control. By default, position control mode is used, and hybrid control mode is supported

  // Joint pd control - position control method, namely, issuing kp, kd, goal_ angle，goal_ Vel, feedforward force tor=0, joint end completes joint pd control closed-loop
  if(true){
    
    cmd.joint_cmd[3 * leg_side].kp = kp_hipx;
    cmd.joint_cmd[3 * leg_side + 1].kp = kp_hipy;
    cmd.joint_cmd[3 * leg_side + 2].kp = kp_knee;
    cmd.joint_cmd[3 * leg_side].kd = kd_hipx;
    cmd.joint_cmd[3 * leg_side + 1].kd = kd_hipy;
    cmd.joint_cmd[3 * leg_side + 2].kd = kd_knee;
    cmd.joint_cmd[3 * leg_side].pos = goal_angle[0];
    cmd.joint_cmd[3 * leg_side + 1].pos = goal_angle[1];
    cmd.joint_cmd[3 * leg_side + 2].pos = goal_angle[2];
    cmd.joint_cmd[3 * leg_side].vel = goal_vel[0];
    cmd.joint_cmd[3 * leg_side + 1].vel = goal_vel[1];
    cmd.joint_cmd[3 * leg_side + 2].vel = goal_vel[2];
    for (int i = 0; i < 12; i++) {
      cmd.joint_cmd[i].tor = 0;
    }
  } else{
    // Joint pd control - force control method, i.e. kp, kd, goal_ angle，goal_ Set level to 0
    // The upper layer obtains real-time joint data and compares it with the target, performs pd control, calculates the expected torque of each joint, and sends it as a feedforward force tor to the joint end, which directly executes the target torque
    cmd.joint_cmd[3 * leg_side].kp = 0;
    cmd.joint_cmd[3 * leg_side + 1].kp = 0;
    cmd.joint_cmd[3 * leg_side + 2].kp = 0;
    cmd.joint_cmd[3 * leg_side].kd = 0;
    cmd.joint_cmd[3 * leg_side + 1].kd = 0;
    cmd.joint_cmd[3 * leg_side + 2].kd = 0;
    cmd.joint_cmd[3 * leg_side].pos = 0;
    cmd.joint_cmd[3 * leg_side + 1].pos = 0;
    cmd.joint_cmd[3 * leg_side + 2].pos = 0;
    cmd.joint_cmd[3 * leg_side].vel = 0;
    cmd.joint_cmd[3 * leg_side + 1].vel = 0;
    cmd.joint_cmd[3 * leg_side + 2].vel = 0;
    cmd.joint_cmd[3* leg_side].tor = 60 * (goal_angle[0] - data.joint_data[3* leg_side].pos)
                                    + 0.7 * (goal_vel[0] - data.joint_data[3* leg_side].vel);
    cmd.joint_cmd[3* leg_side+1].tor = 80 * (goal_angle[1] - data.joint_data[3* leg_side+1].pos)
                                    + 0.7 * (goal_vel[1] - data.joint_data[3* leg_side+1].vel);
    cmd.joint_cmd[3* leg_side+2].tor = 80 * (goal_angle[2] - data.joint_data[3* leg_side+2].pos)
                                    + 0.7 * (goal_vel[2] - data.joint_data[3* leg_side+2].vel);
  }
}

/**
 * @brief Interpolation to find the path point, i.e. the target angle for each control cycle
 * @param init_pos 
 * @param init_vel 
 * @param goal_pos 
 * @param goal_vel 
 * @param run_time 
 * @param cycle_time Control cycle, default is 1ms
 * @param total_time 
 * @param sub_goal_pos Target angle for the control cycle
 * @param sub_goal_pos_next Target angle for the next control cycle
 * @param sub_goal_pos_next2 Target angle for the next and next control cycle
 */
void MotionSDKMoveRobot::CubicSpline(double init_pos, double init_vel,
                                double goal_pos, double goal_vel,
                                double run_time, double cycle_time,
                                double total_time, double &sub_goal_pos,
                                double &sub_goal_pos_next,
                                double &sub_goal_pos_next2) {
  double a, b, c, d;
  d = init_pos;
  c = init_vel;
  a = (goal_vel * total_time - 2 * goal_pos + init_vel * total_time +
       2 * init_pos) /
      pow(total_time, 3);
  b = (3 * goal_pos - goal_vel * total_time - 2 * init_vel * total_time -
       3 * init_pos) /
      pow(total_time, 2);

  if (run_time > total_time)
    run_time = total_time;
  sub_goal_pos = a * pow(run_time, 3) + b * pow(run_time, 2) + c * run_time + d;

  if (run_time + cycle_time > total_time)
    run_time = total_time - cycle_time;
  sub_goal_pos_next = a * pow(run_time + cycle_time, 3) +
                      b * pow(run_time + cycle_time, 2) +
                      c * (run_time + cycle_time) + d;

  if (run_time + cycle_time * 2 > total_time)
    run_time = total_time - cycle_time * 2;
  sub_goal_pos_next2 = a * pow(run_time + cycle_time * 2, 3) +
                       b * pow(run_time + cycle_time * 2, 2) +
                       c * (run_time + cycle_time * 2) + d;
}