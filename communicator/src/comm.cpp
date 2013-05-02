/***************************************************************************
 ***************************************************************************
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU     *
 *   Lesser General Public License for more details.                       *
 *                                                                         *
 *   You should have received a copy of the GNU Lesser General Public      *
 *   License along with this library; if not, write to the Free Software   *
 *   Foundation, Inc., 59 Temple Place,                                    *
 *   Suite 330, Boston, MA  02111-1307  USA                                *
 *                                                                         *
 ***************************************************************************/

#include "comm.hpp"

ORO_CREATE_COMPONENT (communicator::Communicator)

namespace communicator
{
using namespace RTT;

Communicator::Communicator(const std::string& name) :
    TaskContext(name)
{
  //add ROS Ports: EventPorts for Inputs, "normal" Ports for outputs
  this->addEventPort("cmd_vel", twist_in).doc("Base Twist Command ROS");
  this->addEventPort("base_current_command", base_cur_in).doc("Base Current Command ROS");
  this->addEventPort("arm_position_command", arm_pos_in).doc("Arm Position Command ROS");
  this->addEventPort("arm_velocity_command", arm_vel_in).doc("Arm Velocity Command ROS");
  this->addEventPort("arm_torques_command", arm_tor_in).doc("Arm Torques Command ROS");
  this->addEventPort("grip_position_command", gri_pos_in).doc("Gripper Position Command ROS");

  this->addPort("joint_states", j_state_out).doc("joint_states message ROS");
  this->addPort("odom", odom_out).doc("Odometry message ROS");

  //add Orocos Ports: EventPorts for Inputs, "normal" Ports for outputs
  this->addEventPort("arm_motor_states", arm_motor_states).doc("Arm Motor States Orocos");
  this->addEventPort("joint_states_in", j_state_in).doc("joint_states message Orocos");
  this->addEventPort("arm_events", arm_events).doc("Arm Events Orocos");
  this->addEventPort("odom_in", odom_in).doc("Odometry message Orocos");
  this->addEventPort("base_motor_states", base_motor_states).doc("Base Motor States Orocos");
  this->addEventPort("base_events", base_events).doc("Base Events Orocos");

  this->addPort("arm_control_mode", arm_control_mode).doc("Arm Control Mode Orocos");
  this->addPort("base_control_mode", base_control_mode).doc("Base Control Mode Orocos");
  this->addPort("cmd_vel_out", twist_out).doc("Base Twist Command Orocos");
  this->addPort("base_current_command_oro", base_cur_out).doc("Base Current Command Orocos");
  this->addPort("arm_pos_com_oro", arm_pos_out).doc("Arm Position Command Orocos");
  this->addPort("arm_vel_com_oro", arm_vel_out).doc("Arm Velocity Command Orocos");
  this->addPort("arm_tor_com_oro", arm_tor_out).doc("Arm Torques Command Orocos");
  this->addPort("arm_gri_pos_oro", gri_pos_out).doc("Gripper Position Command Orocos");
}
void Communicator::updateHook()
{
  if(twist_in.read(twist_msg)==NewData)
  {
    control_mode.data="Velocity";
    base_control_mode.write(control_mode);
    twist_out.write(twist_msg);
  }
}

bool Communicator::configureHook()
{
  return true;
}

} //namespace
