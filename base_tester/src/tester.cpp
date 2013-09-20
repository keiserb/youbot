#include "tester.hpp"

using namespace std;

base_tester::base_tester(ros::NodeHandle& nh)
{
  base_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  opti_sub = nh.subscribe("/optitrack", 1, &base_tester::optiCallback, this);
}

void base_tester::optiCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  double roll, pitch, yaw;
  geometry_msgs::PoseStamped pos = *msg;
  btQuaternion rot(pos.pose.orientation.x,pos.pose.orientation.y,pos.pose.orientation.z,pos.pose.orientation.w);
  tf::Matrix3x3(rot).getRPY(roll,pitch,yaw);
  opti_pos.linear.x = pos.pose.position.x;
  opti_pos.linear.y = pos.pose.position.y;
  opti_pos.linear.z = pos.pose.position.z;
  opti_pos.angular.x = roll;
  opti_pos.angular.y = pitch;
  opti_pos.angular.z = yaw;
}

void base_tester::getTF(geometry_msgs::Twist & base_pos_odom, geometry_msgs::Twist & base_pos_opti)
{
  double roll, pitch, yaw;
  tf::StampedTransform od_bf;
  try
  {
    lr.lookupTransform("odom", "base_footprint", ros::Time(0), od_bf);
  }
  catch (tf::TransformException ex)
  {
    ROS_ERROR("%s", ex.what());
  }
  tf::Vector3 trans = od_bf.getOrigin();
  tf::Quaternion rot = od_bf.getRotation();
  tf::Matrix3x3(rot).getRPY(roll, pitch, yaw);

  base_pos_odom.linear.x = trans.x();
  base_pos_odom.linear.y = trans.y();
  base_pos_odom.linear.z = trans.z();
  base_pos_odom.angular.x = roll;
  base_pos_odom.angular.y = pitch;
  base_pos_odom.angular.z = yaw;

  base_pos_opti.linear.x=opti_pos.linear.x-init_opti_pos.linear.x;
  base_pos_opti.linear.y=opti_pos.linear.y-init_opti_pos.linear.y;
  base_pos_opti.linear.z=opti_pos.linear.z-init_opti_pos.linear.z;
  base_pos_opti.angular.x=opti_pos.angular.x-init_opti_pos.angular.x;
  base_pos_opti.angular.y=opti_pos.angular.y-init_opti_pos.angular.y;
  base_pos_opti.angular.z=opti_pos.angular.z-init_opti_pos.angular.z;
}

void base_tester::move_base()
{
  int x;
  ros::spinOnce();
  init_opti_pos=opti_pos;
  ros::Rate loop_rate(50);
  geometry_msgs::Twist odom_position, opti_position;
  geometry_msgs::Twist mov;
  mov.linear.x = 0.1;
  mov.linear.y = 0.0;
  mov.linear.z = 0.0;
  mov.angular.x = 0.0;
  mov.angular.y = 0.0;
  mov.angular.z = 0.0;
  ros::spinOnce();
  loop_rate.sleep();
  ros::spinOnce();
  getTF(odom_position, opti_position);
  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  ROS_INFO("Moving base 1m in x direction, ready?");
  cin >> x;
  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  while(odom_position.linear.x < 1 )
  {
    base_pub.publish(mov);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    getTF(odom_position, opti_position);
  }
  mov.linear.x = 0.0;
  base_pub.publish(mov);

  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  ROS_INFO("Moving base -1m in x direction, ready?");
  cin >> x;
  mov.linear.x = -0.1;
  while(odom_position.linear.x > 0 )
  {
    base_pub.publish(mov);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    getTF(odom_position, opti_position);
  }
  mov.linear.x = 0.0;
  base_pub.publish(mov);
  mov.linear.y = 0.1;
  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  ROS_INFO("Moving base 1m in y direction, ready?");
  cin >> x;
  while(odom_position.linear.y < 1 )
  {
    base_pub.publish(mov);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    getTF(odom_position, opti_position);
  }
  mov.linear.y = 0.0;
  base_pub.publish(mov);
  mov.linear.y = -0.1;
  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  ROS_INFO("Moving base -1m in y direction, ready?");
  cin >> x;
  while(odom_position.linear.y > 0 )
  {
    base_pub.publish(mov);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    getTF(odom_position, opti_position);
  }
  mov.linear.y = 0.0;
  base_pub.publish(mov);
  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  mov.angular.z = 0.1;
  ROS_INFO("Rotating Base by 90°, ready?");
  cin >> x;
  while(odom_position.angular.z < M_PI/2 )
  {
    base_pub.publish(mov);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    getTF(odom_position, opti_position);
  }
  mov.angular.z = 0.0;
  base_pub.publish(mov);
  ROS_INFO("Current Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("Current Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
  mov.angular.z = -0.1;
  ROS_INFO("Rotating Base by -90°, ready?");
  cin >> x;
  while(odom_position.angular.z > 0 )
  {
    base_pub.publish(mov);
    ros::spinOnce();
    loop_rate.sleep();
    ros::spinOnce();
    getTF(odom_position, opti_position);
  }
  mov.angular.z = 0;
  base_pub.publish(mov);
  ROS_INFO("End Position Odom: x: %f \t y: %f \t yaw: %f", odom_position.linear.x, odom_position.linear.y, odom_position.angular.z);
  ROS_INFO("End Position Opti: x: %f \t y: %f \t yaw: %f", opti_position.linear.x, opti_position.linear.y, opti_position.angular.z);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_commander");
  ros::NodeHandle nh;
  base_tester bt(nh);
  bt.move_base();
  return 0;
}

