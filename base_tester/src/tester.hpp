/*
 * tester.hpp
 *
 *  Created on: Sep 20, 2013
 *      Author: keiserb
 */

#ifndef TESTER_HPP_
#define TESTER_HPP_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

class base_tester
{
public:
  base_tester(ros::NodeHandle& nh);
  void move_base();
  ros::Publisher base_pub;
  ros::Subscriber opti_sub;
  tf::TransformListener lr;
private:
  geometry_msgs::Twist opti_pos;
  geometry_msgs::Twist init_opti_pos;
  void optiCallback(geometry_msgs::PoseStamped::ConstPtr& msg);
  void getTF(geometry_msgs::Twist & base_pos_odom, geometry_msgs::Twist & base_pos_opti);
};

#endif /* TESTER_HPP_ */
