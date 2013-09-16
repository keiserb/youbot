/*
 * grav.cpp
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#include "grav.hpp"

using namespace KDL;

GravityCompensator::GravityCompensator(ros::NodeHandle& nh) :
    DOF(5)
{
  // arm joints (always 5)
  m_joint_state.name.assign(5, "0        10       20"); // presized string
  m_joint_state.position.resize(5);
  m_joint_state.velocity.resize(5);
  m_joint_state.effort.resize(5);

  m_joint_torques.torques.resize(5);
  //subscribe to ports. At the moment no messages are received from wrench and accelerations
  joint_state_sub = nh.subscribe("/joint_states", 1, &GravityCompensator::jointstateCallback, this);
  torque_command_pub = nh.advertise<brics_actuator::JointTorques>("/arm_1/arm_controller/torques_command", 1);
}

bool GravityCompensator::initialize()
{
  DOF = 5;
  m_q.resize(DOF);
  m_qdot.resize(DOF);
  m_qdotdot.resize(DOF);
  m_torques.resize(DOF);

  SetToZero(m_q);
  SetToZero(m_qdot);
  SetToZero(m_qdotdot);
  SetToZero(m_torques);

  return true;
}

brics_actuator::JointTorques GravityCompensator::generate_joint_torque_msg(KDL::JntArray arr)
{
  std::stringstream jointName;
  m_joint_torques.torques.clear();

  for (int i = 0; i < 5; i++)
  {
    brics_actuator::JointValue joint;
    joint.value = arr.data[i];
    joint.unit = boost::units::to_string(boost::units::si::newton_meter);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    m_joint_torques.torques.push_back(joint);
  }
  return m_joint_torques;
}

void GravityCompensator::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int size=0;
  for (int j = 0; j < msg->position.size(); j++)
  {
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      if (msg->name[j] == joint_names[i])
      {
        m_joint_state.position[i] = msg->position[j];
        size++;
      }
    }
  }
  if (size == YOUBOT_NR_OF_JOINTS)
  {
    for (int i = 0; i < 5; i++)
    {
      //translate values into Orocos positions
      m_q(0) = m_joint_state.position[0] - joint_offsets[0];
      m_q(1) = m_joint_state.position[1] - joint_offsets[1];
      m_q(2) = joint_offsets[2] - m_joint_state.position[2];
      m_q(3) = m_joint_state.position[3] - joint_offsets[3];
      m_q(4) = m_joint_state.position[4] - joint_offsets[4];
      //std::cout << "position joint : " << i << "\t" << m_joint_state.position[i] << "\t - offset: \t" << m_q(i) << std::endl;
    }
    //only m_q is non zero, m_torques gets written.
    calcTorques(m_q, m_qdot, m_qdotdot, m_torques);
    //Ros component negates torque values for joints with negative direction (all joints except joint 3)
    m_torques(2) = - m_torques(2);

    torque_command_pub.publish(generate_joint_torque_msg(m_torques));
  }
  else
  {
    ROS_WARN("NO JOINT STATES FOR YOUBOT ARM RECEIVED");
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "gravity_compensator");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  GravityCompensator gcomp(nh);

  gcomp.initialize();

  ros::spin();

  return 0;
}
