#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

void setup_trajectory(control_msgs::FollowJointTrajectoryGoal & goal)
{
  goal.trajectory.joint_names.push_back("arm_joint_1");
  goal.trajectory.joint_names.push_back("arm_joint_2");
  goal.trajectory.joint_names.push_back("arm_joint_3");
  goal.trajectory.joint_names.push_back("arm_joint_4");
  goal.trajectory.joint_names.push_back("arm_joint_5");

  trajectory_msgs::JointTrajectoryPoint point;
  point.time_from_start = ros::Duration(2.0);
  point.positions.push_back(0.5);
  point.velocities.push_back(0);
  point.accelerations.push_back(0);
  point.positions.push_back(0.5);
  point.velocities.push_back(0);
  point.accelerations.push_back(0);
  point.positions.push_back(-0.5);
  point.velocities.push_back(0);
  point.accelerations.push_back(0);
  point.positions.push_back(0.5);
  point.velocities.push_back(0);
  point.accelerations.push_back(0);
  point.positions.push_back(0.5);
  point.velocities.push_back(0);
  point.accelerations.push_back(0);
  goal.trajectory.points.push_back(point);
  point.time_from_start = ros::Duration(4.0);
  point.positions.clear();
  point.positions.push_back(0.6);
  point.positions.push_back(0.6);
  point.positions.push_back(-0.6);
  point.positions.push_back(0.6);
  point.positions.push_back(0.6);
  goal.trajectory.points.push_back(point);

  control_msgs::JointTolerance gt;
  control_msgs::JointTolerance pt;

  gt.position = 0.01;
  gt.velocity = 0.01;
  gt.acceleration = 0.01;

  pt=gt;

  goal.goal_tolerance.push_back(gt);
  goal.path_tolerance.push_back(pt);

  goal.goal_time_tolerance.sec = 1;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_fibonacci");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac(
      "/arm_1/arm_controller/follow_joint_trajectory/", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer(); //will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  control_msgs::FollowJointTrajectoryGoal goal;

  setup_trajectory(goal);
  goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(2.0);
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

  //exit
  return 0;
}
