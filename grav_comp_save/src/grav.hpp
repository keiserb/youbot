/*
 * grav.hpp
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#ifndef GRAV_HPP_
#define GRAV_HPP_

#include <iostream>

#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainidsolver_recursive_newton_euler.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>

#include <geometry_msgs/Wrench.h>
#include <motion_control_msgs/JointEfforts.h>
#include <motion_control_msgs/JointAccelerations.h>

#include <tf_conversions/tf_kdl.h>


bool include_base;
bool include_base_local;

unsigned int DOF;
std::string prop_urdf_model;
std::vector<double> force_gain;

sensor_msgs::JointState m_joint_state;
motion_control_msgs::JointEfforts m_joint_efforts;
motion_control_msgs::JointAccelerations m_joint_accelerations;
geometry_msgs::Twist m_base_twist;

KDL::JntArray m_q, m_qdot, m_qdotdot, m_torques;

geometry_msgs::Wrench m_wrench_msg;
KDL::Wrenches m_wrenches;
KDL::Chain m_chain;

boost::shared_ptr<KDL::ChainIdSolver_RNE> m_id_solver;




#endif /* GRAV_HPP_ */
