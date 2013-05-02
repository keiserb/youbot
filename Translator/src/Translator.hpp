/*
 * Translater.hpp
 *
 *  Created on: Apr 30, 2013
 *      Author: keiserb
 */

#ifndef TRANSLATER_HPP_
#define TRANSLATER_HPP_

#include "ros/ros.h"
#include "rpg_youbot_common.h"

#include <brics_actuator/JointPositions.h>

#include <sensor_msgs/JointState.h>

#include <geometry_msgs/Twist.h>

#include <youbot_msgs/motor_states.h>
#include <youbot_msgs/driver_state.h>

#include <nav_msgs/Odometry.h>

#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>

using namespace std;

geometry_msgs::Twist command;

brics_actuator::JointPositions jointpos;

std_msgs::Int32MultiArray current_command;
std_msgs::String arm_control_mode;
std_msgs::String base_control_mode;

ros::Publisher pos_command_oro_pub;
ros::Publisher velocity_command_oro_pub;
ros::Publisher torques_command_oro_pub;
ros::Publisher arm_control_mode_oro_pub;
ros::Publisher gripper_command_oro_pub;
ros::Publisher pos_command_oro_pub;
ros::Publisher pos_command_oro_pub;
ros::Publisher pos_command_oro_pub;
ros::Publisher pos_command_oro_pub;
ros::Publisher pos_command_oro_pub;
ros::Publisher pos_command_oro_pub;




#endif /* TRANSLATER_HPP_ */
