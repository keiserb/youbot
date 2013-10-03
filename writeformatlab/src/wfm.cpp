/*
 * wfm.cpp
 *
 *  Created on: Aug 20, 2013
 *      Author: keiserb
 */

#include "wfm.hpp"

using namespace std;

wfm::wfm(ros::NodeHandle& nh)
{
  ztar=0;
  error = 0.001;
  counter = 0;
  loop = 0;
  pos = new double[5];
  arm_cmd.positions.resize(5);
  jpos_eig.setZero(YOUBOT_NR_OF_JOINTS);
  jpos_sub = nh.subscribe("/joint_states", 1, &wfm::jointStateCallback, this);
  jpos_pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);
  jvel_pub = nh.advertise<brics_actuator::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);
  pose_pub = nh.advertise<geometry_msgs::Pose>("/torque_controller/target_pos_CS", 1);
  cmd_mode_pub = nh.advertise<std_msgs::String>("/torque_controller/command_mode", 1);
  solve_fully_constrained_ik_client = nh.serviceClient<ik_solver_service::SolveFullyConstrainedIK>(
      "solve_fully_constrained_ik");
  cs2cs_client = nh.serviceClient<trajectory_generator::CStoCS>("From_CS_to_CS");
}

void wfm::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int size = 0;
  for (int j = 0; j < msg->position.size(); j++)
  {
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      if (msg->name[j] == joint_names[i])
      {
        j_pos[i] = msg->position[j];
        jpos_eig[i] = j_pos[i];
        size++;
      }
    }
  }
  if (size != YOUBOT_NR_OF_JOINTS)
  {
    ROS_WARN("incomplete joint state message");
  }
}

void wfm::lowerdirect(geometry_msgs::Pose object)
{
  if (positionReached(solve_ik(object)))
  {
    object.position.z = object.position.z - 0.08;
    publishArmCommand(object);
  }
}

void wfm::lowermmstep(geometry_msgs::Pose object)
{
  object.position.z = object.position.z - 0.001 * counter;
  Eigen::VectorXd temp(5);
  temp=youbot2matlab(jpos_eig);
  getCartPos(temp,cart_pos);
  Eigen::Vector3d tra= cart_pos.translation();
  ROS_INFO("Z Position: %f", object.position.z);
  ROS_INFO("Counter: %d" , counter);
  ROS_INFO("act Z Position: %f", tra.z());
  for(int i=0; i<5; i++)
  {
    pos[i]=arm_cmd.positions[i].value;
  }
  if ((positionReached(solve_ik(object)) || positionReached(pos)) && counter <80 && tra.z() > ztar)
  {
    object.position.z = object.position.z - 0.001 * (counter + 1);
    arm_cmd=rpg_youbot_common::generate_joint_position_msg(solve_ik(object));
    jpos_pub.publish(arm_cmd);
    counter++;
  }
  else if(tra.z() <= ztar)
  {
    ros::shutdown();
  }
  else
  {
    if (loop % 50 == 0)
    {
      jpos_pub.publish(arm_cmd);
    }
  }
}

void wfm::lowervel(geometry_msgs::Pose object)
{
  Eigen::VectorXd temp(5);
  temp=youbot2matlab(jpos_eig);
  getCartPos(temp,cart_pos);
  Eigen::Vector3d tra= cart_pos.translation();
  ROS_INFO("act Z Position: %f", tra.z());
  if (positionReached(solve_ik(object)))
  {
    double* oldpos, *newpos;
    double vel[YOUBOT_NR_OF_JOINTS];
    oldpos = solve_ik(object);
    object.position.z = object.position.z - 0.08;
    newpos = solve_ik(object);
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      vel[i] = (newpos[i] - oldpos[i]) / 4;
    }
    jvel_pub.publish(rpg_youbot_common::generate_joint_velocity_msg(vel));
    counter++;
  }
  else if (tra.z() < ztar || counter > 200)
  {
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      double vel[YOUBOT_NR_OF_JOINTS] = {0, 0, 0, 0, 0};
      jvel_pub.publish(rpg_youbot_common::generate_joint_velocity_msg(vel));
      ros::shutdown();
    }
  }
  else if (counter > 0)
  {
    counter++;
  }
}

void wfm::lowervelfb(geometry_msgs::Pose object)
{
  Eigen::VectorXd temp(5);
  temp=youbot2matlab(jpos_eig);
  getCartPos(temp,cart_pos);
  Eigen::Vector3d tra= cart_pos.translation();
  double vel[YOUBOT_NR_OF_JOINTS];
  ROS_INFO("act Z Position: %f", tra.z());
  if (positionReached(solve_ik(object)))
  {
    double* oldpos, *newpos;
    oldpos = solve_ik(object);
    object.position.z = object.position.z - 0.08;
    newpos = solve_ik(object);
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      t_pos[i] = newpos[i];
      vel[i] = (t_pos[i] - oldpos[i]) / 2;
    }
    jvel_pub.publish(rpg_youbot_common::generate_joint_velocity_msg(vel));
    counter++;
  }
  else if (tra.z() < ztar)
  {
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      double vel[YOUBOT_NR_OF_JOINTS] = {0, 0, 0, 0, 0};
      jvel_pub.publish(rpg_youbot_common::generate_joint_velocity_msg(vel));
      ros::shutdown();
    }
  }
  else if (counter > 0 && counter < 100)
  {

    object.position.z = object.position.z - 0.08;
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      vel[i] = (t_pos[i] - j_pos[i])/(2-counter/50);
    }
    jvel_pub.publish(rpg_youbot_common::generate_joint_velocity_msg(vel));
    counter++;
  }
}

void wfm::lowertorque(geometry_msgs::Pose & object, trajectory_msgs::JointTrajectory & traj)
{
  if (positionReached(solve_ik(object)) && counter == 0)
  {
    actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("torque_control", true);
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("Action server started, sending goal.");

    torque_control::torque_trajectoryGoal goal;
    goal.trajectory = traj;

    ac.sendGoal(goal);

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
      actionlib::SimpleClientGoalState state = ac.getState();
      ROS_INFO("Action finished: %s", state.toString().c_str());
    }
    else
      ROS_INFO("Action did not finish before the time out.");
    counter++;
  }
}

void wfm::publishArmCommand(geometry_msgs::Pose object)
{
  jpos_pub.publish(rpg_youbot_common::generate_joint_position_msg(solve_ik(object)));
}

double* wfm::solve_ik(geometry_msgs::Pose object)
{
  double *response = new double[YOUBOT_NR_OF_JOINTS];
  double roll, pitch, yaw;
  ik_solver_service::SolveFullyConstrainedIK fc_srv;
  btQuaternion q(object.orientation.x, object.orientation.y, object.orientation.z, object.orientation.w);
  tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
  fc_srv.request.id = 1;
  fc_srv.request.pitch = M_PI / 2; //grab object from top
  fc_srv.request.des_position[0] = object.position.x;
  fc_srv.request.des_position[1] = object.position.y;
  fc_srv.request.des_position[2] = object.position.z;
  fc_srv.request.des_normal[0] = -sin(yaw);
  fc_srv.request.des_normal[1] = cos(yaw);
  fc_srv.request.des_normal[2] = 0;
  solve_fully_constrained_ik_client.call(fc_srv);
  if (fc_srv.response.feasible)
  {
    response[0] = fc_srv.response.joint_angles[0];
    response[1] = fc_srv.response.joint_angles[1];
    response[2] = fc_srv.response.joint_angles[2];
    response[3] = fc_srv.response.joint_angles[3];
    response[4] = fc_srv.response.joint_angles[4];
  }
  else
  {
    ROS_WARN("POSITION NOT FEASIBLE");
  }
  return response;
}

double* wfm::getJPos()
{
  double *response = new double[YOUBOT_NR_OF_JOINTS];
  for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
  {
    response[i] = j_pos[i];
  }
  return response;
}

bool wfm::positionReached(double * jpos)
{
  double err = 0;
  //cout << "error: ";
  for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
  {
    //cout << fabs(j_pos[i] - jpos[i]) << "\t";
    if (fabs(j_pos[i] - jpos[i]) < error)
    {
      err++;
    }
  }
  //cout << endl;
  if (err != YOUBOT_NR_OF_JOINTS)
  {
    return false;
  }
  return true;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "writeformatlab");
  ros::NodeHandle nh;
  ros::Rate loop_rate(50);
  ofstream jp;
  wfm wm(nh);
  double* pos;
  jp.open("joint_pos.txt", std::ios::out);
  geometry_msgs::Pose object;
  object.position.x = 0.0;
  //pos 1
  object.position.y = -0.25;
  object.position.z = -0.1;
  wm.ztar=object.position.z;
  //pos 2
  //object.position.y = -0.2;
  //object.position.z = 0.08;
  trajectory_generator::CStoCS cs2cs;
  trajectory_msgs::JointTrajectory traj;
  Eigen::Matrix3d rot;
  rot.col(0) << 0, 0, -1;
  rot.col(1) << 0, 1, 0;
  rot.col(2) << 1, 0, 0;
  Eigen::Quaterniond grip(rot);
  object.orientation.x = grip.x();
  object.orientation.y = grip.y();
  object.orientation.z = grip.z();
  object.orientation.w = grip.w();
  cs2cs.request.end_pos = object;
  object.position.z = -0.02;
  cs2cs.request.start_pos = object;
  cs2cs.request.start_vel = 0.0;
  cs2cs.request.end_vel = 0.0;
  cs2cs.request.max_vel = 0.05;
  cs2cs.request.max_acc = 0.5;

  sleep(1);
  ros::spinOnce();
  wm.publishArmCommand(object);

  wm.cs2cs_client.call(cs2cs);
  if (cs2cs.response.feasible)
  {
    traj = cs2cs.response.trajectory;
  }
  loop_rate.sleep();
  ros::spinOnce();
  while (ros::ok())
  {
    wm.lowervelfb(object);
    pos = wm.getJPos();

    jp << pos[0] << "\t" << pos[1] << "\t" << pos[2] << "\t" << pos[3] << "\t" << pos[4] << "\t" << endl;

    wm.loop++;
    loop_rate.sleep();
    ros::spinOnce();
  }
  return 0;

  return 0;
}

