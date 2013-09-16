/*
 * grav.hpp
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#ifndef GRAV_HPP_
#define GRAV_HPP_

#include <iostream>

#include <ros/ros.h>
//#include <rpg_youbot_common.h>
#include "YoubotArmDynamicsSymbolic.hpp"
#include "YoubotArmModel.hpp"
#include "YoubotJoints.hpp"

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <boost/units/systems/si.hpp>

#include <brics_actuator/JointTorques.h>
#include <sensor_msgs/JointState.h>

#include <tf_conversions/tf_kdl.h>

class GravityCompensator
{
public:
  //Constructor
  GravityCompensator(ros::NodeHandle& nh);
  //Destructor
  //~GravityCompensator();
  //define functions
  bool initialize();
  brics_actuator::JointTorques generate_joint_torque_msg(KDL::JntArray arr);
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  //define publisher
  ros::Publisher torque_command_pub;
  //define subscribers
  ros::Subscriber joint_state_sub;

  KDL::JntArray m_q, m_qdot, m_qdotdot, m_torques;

  unsigned int DOF;

  sensor_msgs::JointState m_joint_state;
  brics_actuator::JointTorques m_joint_torques;
};
#endif /* GRAV_HPP_ */
