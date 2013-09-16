#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include "YoubotArmDynamicsSymbolic.hpp"
#include "YoubotJoints.hpp"

#include <brics_actuator/JointTorques.h>
#include <sensor_msgs/JointState.h>

#include <boost/units/systems/si.hpp>

#include <tf_conversions/tf_kdl.h>
#include <iostream>

using namespace KDL;
using namespace std;

unsigned int DOF = 5;
KDL::JntArray m_q, m_qdot, m_qdotdot, m_torques_old, m_torques_new, m_torques_internet;
KDL::Tree old_tree;
KDL::Chain old_chain;
KDL::Tree new_tree;
KDL::Chain new_chain;
//define publisher
ros::Publisher torque_command_pub;
//define subscribers
ros::Subscriber joint_state_sub;

bool getOldTorques(KDL::Chain& m_chain, KDL::JntArray m_q, KDL::JntArray m_qdot, KDL::JntArray m_qdotdot,
                   KDL::JntArray& m_torques)
{
  KDL::Wrenches m_wrenches;
  boost::shared_ptr<KDL::ChainIdSolver_RNE> m_id_solver;

  m_wrenches.resize(m_chain.getNrOfSegments());
  SetToZero(m_wrenches[m_wrenches.size() - 1]);

  m_id_solver.reset(new ChainIdSolver_RNE(m_chain, Vector(0, 0, -9.81)));

  int ret = 0;
  ret = m_id_solver->CartToJnt(m_q, m_qdot, m_qdotdot, m_wrenches, m_torques);
  if (ret < 0)
  {
    printf("Could not calculate inverse dynamics: %i\n ", ret);
  }
  return ret;
}

bool getNewTorques(KDL::Chain& m_chain, KDL::JntArray m_q, KDL::JntArray m_qdot, KDL::JntArray m_qdotdot,
                   KDL::JntArray& m_torques)
{
  KDL::Wrenches m_wrenches;
  boost::shared_ptr<KDL::ChainIdSolver_RNE> m_id_solver;
  m_wrenches.resize(m_chain.getNrOfSegments());
  SetToZero(m_wrenches[m_wrenches.size() - 1]);

  m_q(0) = m_q(0) + joint_offsets[0];
  m_q(1) = m_q(1) + joint_offsets[1];
  m_q(2) = joint_offsets[2] - m_q(2);
  m_q(3) = m_q(3) + joint_offsets[3];
  m_q(4) = m_q(4) + joint_offsets[4];

  m_id_solver.reset(new ChainIdSolver_RNE(m_chain, Vector(0, 0, -9.81)));

  int ret = 0;
  ret = m_id_solver->CartToJnt(m_q, m_qdot, m_qdotdot, m_wrenches, m_torques);
  if (ret < 0)
  {
    printf("Could not calculate inverse dynamics: %i\n ", ret);
  }
  m_torques(2)=-m_torques(2);
  return ret;
}

void getInternetTorques(KDL::JntArray m_q, KDL::JntArray m_qdot, KDL::JntArray m_qdotdot, KDL::JntArray& m_torques)
{
  calcTorques(m_q, m_qdot, m_qdotdot, m_torques);
}

brics_actuator::JointTorques generate_joint_torque_msg(KDL::JntArray arr)
{
  brics_actuator::JointTorques m_joint_torques;
  m_joint_torques.torques.resize(5);
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

void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  sensor_msgs::JointState m_joint_state;
  // arm joints (always 5)
  m_joint_state.name.assign(5, "0        10       20"); // presized string
  m_joint_state.position.resize(5);
  m_joint_state.velocity.resize(5);
  m_joint_state.effort.resize(5);
  int size = 0;
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
      //translate values into Torque positions
      m_q(0) = m_joint_state.position[0] - joint_offsets[0];
      m_q(1) = m_joint_state.position[1] - joint_offsets[1];
      m_q(2) = joint_offsets[2] - m_joint_state.position[2];
      m_q(3) = m_joint_state.position[3] - joint_offsets[3];
      m_q(4) = m_joint_state.position[4] - joint_offsets[4];
      //std::cout << "position joint : " << i << "\t" << m_joint_state.position[i] << "\t - offset: \t" << m_q(i) << std::endl;
    }
    /*
     //only m_q is non zero, m_torques gets written.
     getOldTorques(old_chain, m_q, m_qdot, m_qdotdot, m_torques_old);
     //Ros component negates torque values for joints with negative direction (all joints except joint 3)
     m_torques_old(2) = - m_torques_old(2);
     getInternetTorques(m_q, m_qdot, m_qdotdot, m_torques_internet);
     m_torques_internet(2) = - m_torques_internet(2);
     getNewTorques(new_chain, m_q, m_qdot, m_qdotdot, m_torques_new);
     cout << "old" << endl;
     cout << m_torques_old << endl;
     cout << "new" << endl;
     cout << m_torques_new << endl;
     cout << "internet" << endl;
     cout << m_torques_internet << endl;
     torque_command_pub.publish(generate_joint_torque_msg(m_torques_old));*/
  }
  else
  {
    ROS_WARN("NO JOINT STATES FOR YOUBOT ARM RECEIVED");
  }
}

void initialize(ros::NodeHandle& nh)
{
  m_q.resize(DOF);
  m_qdot.resize(DOF);
  m_qdotdot.resize(DOF);
  m_torques_old.resize(DOF);
  m_torques_new.resize(DOF);
  m_torques_internet.resize(DOF);
  SetToZero(m_q);
  SetToZero(m_qdot);
  SetToZero(m_qdotdot);
  SetToZero(m_torques_old);
  SetToZero(m_torques_new);
  SetToZero(m_torques_internet);
  joint_state_sub = nh.subscribe("/joint_states", 1, &jointstateCallback);
  torque_command_pub = nh.advertise<brics_actuator::JointTorques>("/arm_1/arm_controller/torques_command", 1);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "calc_torques");
  ros::NodeHandle nh;
  //increase loop rate when working on real robot
  ros::Rate loop_rate(50);

  KDL::Segment seg;
  KDL::Joint jnt;
  KDL::Vector origin;
  KDL::RigidBodyInertia rbi;
  KDL::Vector cog;
  double mass;

  if (!kdl_parser::treeFromFile("/home/keiserb/myproject/calc_torques/youbot.urdf", old_tree))
  {
    printf("Failed to construct kdl tree \n");
    return false;
  }
  //Fingers don't get added
  if (!old_tree.getChain("arm_link_0", "gripper_palm_link", old_chain))
  {
    printf("Failed to construct subchain from urdf model \n");
    return false;
  }
  if (old_chain.getNrOfJoints() != DOF)
  {
    printf("Wrong number of joints: %i, instead of: %i \n", old_chain.getNrOfJoints(), DOF);
    return false;
  }

  if (!kdl_parser::treeFromFile("/home/keiserb/myproject/calc_torques/youbot_ros.urdf", new_tree))
  {
    printf("Failed to construct kdl tree \n");
    return false;
  }
  //Fingers don't get added
  if (!new_tree.getChain("arm_link_0", "gripper_palm_link", new_chain))
  {
    printf("Failed to construct subchain from urdf model \n");
    return false;
  }
  if (new_chain.getNrOfJoints() != DOF)
  {
    printf("Wrong number of joints: %i, instead of: %i \n", new_chain.getNrOfJoints(), DOF);
    return false;
  }

  initialize(nh);
  //ros::spin();

  std::ifstream rp;
  std::ifstream rv;
  std::ifstream ra;
  std::ofstream ot;
  std::ofstream nt;
  std::ofstream it;
  rp.open("values/req_pos.txt", std::ios::in);
  rv.open("values/req_vel.txt", std::ios::in);
  ra.open("values/req_acc.txt", std::ios::in);
  ot.open("values/old_tor.txt", std::ios::out);
  nt.open("values/new_tor.txt", std::ios::out);
  it.open("values/int_tor.txt", std::ios::out);
  /*
   m_q(0) = -0.3802;
   m_q(1) = -0.4976;
   m_q(2) = -0.5428;
   m_q(3) = -0.3050;
   m_q(4) = -0.1019;

   m_qdot(0) = -0.4883;
   m_qdot(1) = -0.4837;
   m_qdot(2) = 0.0453;
   m_qdot(3) = -0.4652;
   m_qdot(4) = -0.1141;

   m_qdotdot(0) = 0;
   m_qdotdot(1) = 0;
   m_qdotdot(2) = 0;
   m_qdotdot(3) = 0;
   m_qdotdot(4) = 0;

   */

  while (!rp.eof())
  {
    for (int i = 0; i < 5; i++)
    {
      rp >> m_q(i);
      rv >> m_qdot(i);
      ra >> m_qdotdot(i);
    }

    getOldTorques(old_chain, m_q, m_qdot, m_qdotdot, m_torques_old);
    getNewTorques(new_chain, m_q, m_qdot, m_qdotdot, m_torques_new);
    getInternetTorques(m_q, m_qdot, m_qdotdot, m_torques_internet);
    ot << m_torques_old(0) << "\t" << m_torques_old(1) << "\t" << m_torques_old(2) << "\t" << m_torques_old(3) << "\t"
        << m_torques_old(4) << endl;
    nt << m_torques_new(0) << "\t" << m_torques_new(1) << "\t" << m_torques_new(2) << "\t" << m_torques_new(3) << "\t"
        << m_torques_new(4) << endl;
    it << m_torques_internet(0) << "\t" << m_torques_internet(1) << "\t" << m_torques_internet(2) << "\t"
        << m_torques_internet(3) << "\t" << m_torques_internet(4) << endl;

  }

  getOldTorques(old_chain, m_q, m_qdot, m_qdotdot, m_torques_old);
  getNewTorques(new_chain, m_q, m_qdot, m_qdotdot, m_torques_new);
  getInternetTorques(m_q, m_qdot, m_qdotdot, m_torques_internet);
  cout << "old" << endl;
  cout << m_torques_old << endl;
  cout << "new" << endl;
  cout << m_torques_new << endl;
  cout << "internet" << endl;
  cout << m_torques_internet << endl;

  /*
   cout << "Torques: \t";
   for (int i = 0; i < DOF; i++)
   {
   cout << m_torques(i) << "\t";
   }
   cout << endl;
   cout << endl;

   for(int i=0; i<m_chain.getNrOfSegments();i++)
   {
   seg=m_chain.getSegment(i);
   jnt=seg.getJoint();
   origin=jnt.JointOrigin();
   rbi=seg.getInertia();
   cog=rbi.getCOG();
   mass=rbi.getMass();
   cout << "segment " << i+1 << " name: \t" << seg.getName() << endl;
   cout << "CoG: \t x: \t" << cog.data[0] <<"\t y: \t" << cog.data[1] <<"\t z: \t" << cog.data[2] << endl;
   cout << "Mass: \t" << mass << endl;
   cout << "attached to joint : " << jnt.getName() << endl;
   cout << "origin: \t x: \t" << origin.data[0] <<"\t y: \t" << origin.data[1] <<"\t z: \t" << origin.data[2] << endl;
   cout << endl;
   }

   */
  return 0;
}
