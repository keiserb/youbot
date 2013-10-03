/*
 * wfm.hpp
 *
 *  Created on: Aug 20, 2013
 *      Author: keiserb
 */

#ifndef WFM_HPP_
#define WFM_HPP_

#include <ros/ros.h>
#include "sensor_msgs/JointState.h"
#include <iostream>
#include <fstream>
#include "YoubotJoints.hpp"
#include "YoubotArmFKin.hpp"
#include "YoubotArmDynamicsSymbolic.hpp"
#include "rpg_youbot_common.h"
#include "ik_solver_service/SolveFullyConstrainedIK.h"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <tf_conversions/tf_eigen.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_generator/CStoCS.h>
#include <torque_control/torque_trajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <Eigen/Dense>

class wfm
{
public:
  wfm(ros::NodeHandle& nh);
  void lowerdirect(geometry_msgs::Pose object);
  void lowermmstep(geometry_msgs::Pose object);
  void lowervel(geometry_msgs::Pose object);
  void lowervelfb(geometry_msgs::Pose object);
  void lowertorque(geometry_msgs::Pose & object, trajectory_msgs::JointTrajectory & traj);
  void publishArmCommand(geometry_msgs::Pose object);
  ros::ServiceClient cs2cs_client;
  double* getJPos();
  double ztar;
  int counter, loop;
private:
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  double* solve_ik(geometry_msgs::Pose object);
  bool positionReached(double * jpos);
  double j_pos[YOUBOT_NR_OF_JOINTS], t_pos[YOUBOT_NR_OF_JOINTS];
  double error;
  double * pos;
  Eigen::VectorXd jpos_eig;
  Eigen::Affine3d cart_pos;
  brics_actuator::JointPositions arm_cmd;
  ros::Subscriber jpos_sub;
  ros::Publisher jpos_pub;
  ros::Publisher jvel_pub;
  ros::Publisher pose_pub;
  ros::Publisher cmd_mode_pub;
  ros::ServiceClient solve_fully_constrained_ik_client;
};

#endif /* WFM_HPP_ */
