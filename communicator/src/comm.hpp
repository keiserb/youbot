#ifndef COMM_HPP_
#define COMM_HPP_

//needed orocos includes
#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

//include message typekits
#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <youbot_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <brics_actuator/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

#include <boost/units/systems/si.hpp>

#include "YoubotJoints.hpp"

namespace communicator
{

//Problem: How to set control modes!!! -> Not implemented yet.
class Communicator : public RTT::TaskContext
{

public:
  Communicator(const std::string& name); //constructor
//  ~Communicator(); //destructor
      //Ports
      //define InputPorts for ROS Side (ROS stream is defined in .odl file)
  RTT::InputPort<geometry_msgs::Twist> twist_in;
  RTT::InputPort<brics_actuator::JointTorques> base_cur_in;
  RTT::InputPort<brics_actuator::JointPositions> arm_pos_in;
  RTT::InputPort<brics_actuator::JointVelocities> arm_vel_in;
  RTT::InputPort<brics_actuator::JointTorques> arm_tor_in;
  RTT::InputPort<brics_actuator::JointPositions> gri_pos_in;

  //define OutputPorts for ROS Side (ROS stream is defined in .odl file)
  RTT::OutputPort<sensor_msgs::JointState> j_state_out;
  RTT::OutputPort<nav_msgs::Odometry> odom_out;
  RTT::OutputPort<youbot_msgs::motor_states> base_motor_states_ros;
  RTT::OutputPort<youbot_msgs::motor_states> arm_motor_states_ros;

  //define InputPorts for Orocos Side
  RTT::InputPort<nav_msgs::Odometry> odom_in;
  RTT::InputPort<youbot_msgs::motor_states> base_motor_states;
  RTT::InputPort<std::string> base_events;
  RTT::InputPort<youbot_msgs::motor_states> arm_motor_states;
  RTT::InputPort<sensor_msgs::JointState> j_state_in;
  RTT::InputPort<std::string> arm_events;

  //define OutputPorts for Orocos Side
  RTT::OutputPort<std_msgs::String> base_control_mode;
  RTT::OutputPort<std_msgs::String> arm_control_mode;
  RTT::OutputPort<geometry_msgs::Twist> twist_out;
  RTT::OutputPort<std::vector<int> > base_cur_out;
  RTT::OutputPort<motion_control_msgs::JointPositions> arm_pos_out;
  RTT::OutputPort<motion_control_msgs::JointVelocities> arm_vel_out;
  RTT::OutputPort<motion_control_msgs::JointEfforts> arm_tor_out;
  RTT::OutputPort<int> gri_pos_out;

private:
  //define all the messages needed
  geometry_msgs::Twist twist_msg;
  std_msgs::String control_mode;
  motion_control_msgs::JointPositions j_pos_mc;
  motion_control_msgs::JointVelocities j_vel_mc;
  motion_control_msgs::JointEfforts j_eff_mc;
  brics_actuator::JointPositions j_pos_br;
  brics_actuator::JointVelocities j_vel_br;
  brics_actuator::JointTorques j_tor_br;
  brics_actuator::JointPositions m_grip_pos;
  sensor_msgs::JointState m_joint_state;
  nav_msgs::Odometry m_odom;
  youbot_msgs::motor_states base_mot_state;
  youbot_msgs::motor_states arm_mot_state;
  brics_actuator::JointTorques m_base_cur_ros;
  std::vector<int> m_base_cur_oro;

  //define helper functions to translate between different messages
  bool br2mc(brics_actuator::JointPositions jpos);
  bool br2mc(brics_actuator::JointVelocities jvel);
  bool br2mc(brics_actuator::JointTorques jtor);
  std::vector<int> br2vec(brics_actuator::JointTorques base);
  int gripper(brics_actuator::JointPositions grip);

  bool configureHook();
  void updateHook();
};
//class
}//namespace

#endif
