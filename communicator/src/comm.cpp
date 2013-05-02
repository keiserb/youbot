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

ORO_CREATE_COMPONENT(communicator::Communicator)

namespace communicator
{
using namespace RTT;

Communicator::Communicator(const std::string& name) :
    TaskContext(name), m_base_cur_oro(YOUBOT_NR_OF_WHEELS)                 //initialize the vector for base current cmd
{
  //add ROS Ports: EventPorts for Inputs, "normal" Ports for outputs
  //ROS streams get defined in the communicator.lua file
  this->addEventPort("cmd_vel", twist_in).doc("Base Twist Command ROS");
  this->addEventPort("base_current_command", base_cur_in).doc("Base Current Command ROS");
  this->addEventPort("arm_position_command", arm_pos_in).doc("Arm Position Command ROS");
  this->addEventPort("arm_velocity_command", arm_vel_in).doc("Arm Velocity Command ROS");
  this->addEventPort("arm_torques_command", arm_tor_in).doc("Arm Torques Command ROS");
  this->addEventPort("grip_position_command", gri_pos_in).doc("Gripper Position Command ROS");

  this->addPort("joint_states", j_state_out).doc("joint_states message ROS");
  this->addPort("odom", odom_out).doc("Odometry message ROS");
  this->addPort("base_motor_states_out", base_motor_states_ros).doc("Base Motor State Message ROS");
  this->addPort("arm_motor_states_out", arm_motor_states_ros).doc("Arm Motor State Message ROS");
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

  //initialize vectors and arrays
  j_vel_mc.velocities.assign(YOUBOT_NR_OF_JOINTS, 0);
  j_pos_mc.positions.assign(YOUBOT_NR_OF_JOINTS, 0);
  j_eff_mc.efforts.assign(YOUBOT_NR_OF_JOINTS, 0);
  m_base_cur_ros.data.resize(YOUBOT_NR_OF_WHEELS);
}

//converts a brics_actuator Position message to a motion_control Position message
motion_control_msgs::JointPositions Communicator::br2mc(brics_actuator::JointPositions jpos)
{
  for (int j = 0; j < YOUBOT_NR_OF_JOINTS; j++)
  {
    j_pos_mc.positions[j] = jpos.positions[j].value;
  }
  return j_pos_mc;
}

//converts a brics_actuator Velocity message to a motion_control Velocity message
motion_control_msgs::JointVelocities Communicator::br2mc(brics_actuator::JointVelocities jvel)
{
  for (int j = 0; j < YOUBOT_NR_OF_JOINTS; j++)
  {
    j_vel_mc.velocities[j] = jvel.velocities[j].value;
  }
  return j_vel_mc;
}

//converts a brics_actuator Torque message to a motion_control Effort message
motion_control_msgs::JointEfforts Communicator::br2mc(brics_actuator::JointTorques jtor)
{
  for (int j = 0; j < YOUBOT_NR_OF_JOINTS; j++)
  {
    j_eff_mc.efforts[j] = jtor.torques[j].value;
  }
  return j_eff_mc;
}
//converts an array to a vector
std::vector<int> Communicator::ar2vec(std_msgs::Int32MultiArray array)
{
  for (int j = 0; j < YOUBOT_NR_OF_WHEELS; j++)
  {
    m_base_cur_oro[j] = array.data[j];
  }
  return m_base_cur_oro;
}
//Convert the gripper command
int Communicator::gripper(brics_actuator::JointPositions grip)
{
  if(grip.positions[0].value < 0.005)
    return 0;
  else
    return 1;
}

//The update function is repeated as long as the Orocos component is running.
void Communicator::updateHook()
{
  //Handles Base Velocity Commands
  if (twist_in.read(twist_msg) == NewData)
  {
    control_mode.data = "Velocity";
    base_control_mode.write(control_mode);
    twist_out.write(twist_msg);
  }
  //Handles Base Current Commands
  if (base_cur_in.read(m_base_cur_ros))
  {
    control_mode.data = "Current";
    base_control_mode.write(control_mode);
    base_cur_out.write(ar2vec(m_base_cur_ros));
  }
  //Handles Arm Position Commands
  if (arm_pos_in.read(j_pos_br) == NewData)
  {
    control_mode.data = "Positioning";
    arm_control_mode.write(control_mode);
    arm_pos_out.write(br2mc(j_pos_br));
  }
  //Handles Arm Velocity Commands
  if (arm_vel_in.read(j_vel_br) == NewData)
  {
    control_mode.data = "Velocity";
    arm_control_mode.write(control_mode);
    arm_vel_out.write(br2mc(j_vel_br));
  }
  //Handles Arm Torque Commands
  if (arm_tor_in.read(j_tor_br) == NewData)
  {
    control_mode.data = "Current";
    arm_control_mode.write(control_mode);
    arm_tor_out.write(br2mc(j_tor_br));
  }
  //Handles Gripper Commands
  if (gri_pos_in.read(m_grip_pos) == NewData)
  {
    gri_pos_out.write(gripper(m_grip_pos));
  }
  //Handles Joint State Message
  if (j_state_in.read(m_joint_state) == NewData)
  {
    j_state_out.write(m_joint_state);
  }
  //Handles Odometry Message
  if (odom_in.read(m_odom) == NewData)
  {
    odom_out.write(m_odom);
  }
  //Handles Base Motor State Message
  if (base_motor_states.read(base_mot_state) == NewData)
  {
    base_motor_states_ros.write(base_mot_state);
  }
  //Handles Arm Motor State Message
  if (arm_motor_states.read(arm_mot_state) == NewData)
  {
    arm_motor_states_ros.write(arm_mot_state);
  }
}

bool Communicator::configureHook()
{
  return true;
}

} //namespace
