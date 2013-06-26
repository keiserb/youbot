#include <ros/ros.h>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <boost/units/systems/si.hpp>

#include <tf_conversions/tf_kdl.h>
#include <iostream>

using namespace KDL;
using namespace std;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "calc_torques");
  ros::NodeHandle nh;
  //increase loop rate when working on real robot
  ros::Rate loop_rate(10);

  KDL::JntArray m_q, m_qdot, m_qdotdot, m_torques;
  KDL::Wrenches m_wrenches;
  KDL::Chain m_chain;
  KDL::Segment seg;
  KDL::Joint jnt;
  KDL::Vector origin;
  KDL::RigidBodyInertia rbi;
  KDL::Vector cog;
  double mass;

  unsigned int DOF = 5;
  std::string prop_urdf_model;
  boost::shared_ptr<KDL::ChainIdSolver_RNE> m_id_solver;

  m_q.resize(DOF);
  m_qdot.resize(DOF);
  m_qdotdot.resize(DOF);
  m_torques.resize(DOF);
  SetToZero(m_q);
  SetToZero(m_qdot);
  SetToZero(m_qdotdot);
  SetToZero(m_torques);
  if (nh.getParam("/robot_description", prop_urdf_model))
  {
    printf("youbot_description parameter successfully loaded.\n");
  }

  KDL::Tree my_tree;
  if (!kdl_parser::treeFromString(prop_urdf_model, my_tree))
  {
    printf("Failed to construct kdl tree \n");
    return false;
  }
  //Fingers don't get added
  if (!my_tree.getChain("arm_link_0", "gripper_palm_link", m_chain))
  {
    printf("Failed to construct subchain from urdf model \n");
    return false;
  }
  if (m_chain.getNrOfJoints() != DOF)
  {
    printf("Wrong number of joints: %i, instead of: %i \n", m_chain.getNrOfJoints(), DOF);
    return false;
  }

  m_wrenches.resize(m_chain.getNrOfSegments());
  SetToZero(m_wrenches[m_wrenches.size() - 1]);

  m_id_solver.reset(new ChainIdSolver_RNE(m_chain, Vector(0, 0, -9.81)));

  m_q(0) = 0.1;
  m_q(1) = 0.1;
  m_q(2) = 0.1;
  m_q(3) = 0.1;
  m_q(4) = 0.1;

  m_qdot(0) = 0.1;
  m_qdot(1) = 0.1;
  m_qdot(2) = 0.1;
  m_qdot(3) = 0.1;
  m_qdot(4) = 0.1;

  m_qdotdot(0) = 0.1;
  m_qdotdot(1) = 0.1;
  m_qdotdot(2) = 0.1;
  m_qdotdot(3) = 0.1;
  m_qdotdot(4) = 0.1;
  while (ros::ok())
  {
    int ret=0;
    ret = m_id_solver->CartToJnt(m_q, m_qdot, m_qdotdot, m_wrenches, m_torques);
    if (ret < 0)
    {
      printf("Could not calculate inverse dynamics: %i\n ", ret);
      return 0;
    }
    cout << "Torques: \t";
    for (int i = 0; i < DOF; i++)
    {
      cout << m_torques(i) << "\t";
    }
    cout << endl;
    cout << endl;
/*
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
    if(ret>=0)
      return 0;
  }
  return 0;
}
