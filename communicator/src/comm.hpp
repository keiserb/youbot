#ifndef COMM_HPP_
#define COMM_HPP_

#include <rtt/Service.hpp>
#include <rtt/Port.hpp>
#include <rtt/os/TimeService.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Component.hpp>

//#include <rpg_youbot_common.h>

#include <geometry_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>
#include <sensor_msgs/typekit/Types.h>
#include <youbot_msgs/typekit/Types.h>
#include <nav_msgs/typekit/Types.h>
#include <brics_actuator/typekit/Types.h>
#include <motion_control_msgs/typekit/Types.h>

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
  RTT::InputPort<std_msgs::Int32MultiArray> base_cur_in;
  RTT::InputPort<brics_actuator::JointPositions> arm_pos_in;
  RTT::InputPort<brics_actuator::JointVelocities> arm_vel_in;
  RTT::InputPort<brics_actuator::JointTorques> arm_tor_in;
  RTT::InputPort<brics_actuator::JointPositions> gri_pos_in;

  //define OutputPorts for ROS Side (ROS stream is defined in .odl file)
  RTT::OutputPort<sensor_msgs::JointState> j_state_out;
  RTT::OutputPort<nav_msgs::Odometry> odom_out;

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
  geometry_msgs::Twist twist_msg;
  std_msgs::String control_mode;


  bool configureHook();
//  bool startHook();
  void updateHook();
//  void stopHook();
//  void cleanupHook();
};
//class
}//namespace

#endif
