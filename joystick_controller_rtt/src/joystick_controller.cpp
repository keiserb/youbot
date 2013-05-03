#include "ros/ros.h"
#include <algorithm>

#include "rpg_youbot_common.h"
using namespace rpg_youbot_common;

#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

#include <motion_control_msgs/typekit/Types.h>
#include <std_msgs/typekit/Types.h>

ros::Publisher velocity_command_pub;
ros::Publisher gripper_position_pub;
//ros::Publisher joint_position_pub;
ros::Publisher joint_velocity_pub;
ros::Publisher arm_control_mode_pub; 
ros::Publisher base_control_mode_pub;         

double linear_velocity_gain;
double angular_velocity_gain;

bool left_turn_activated, right_turn_activated;
bool quadratic_control;
bool active_control_flag;
bool STOPPED_BASE;
bool STOPPED_ARM;
bool joint_selection_button_released;

int nr_of_wheels = 4;
int nr_of_joints = 5;

std_msgs::Int32 gripper_cmd;
std_msgs::String control_mode;
motion_control_msgs::JointVelocities jvel_msg;
ros::Time last_signal;
//bool joints_initialized = false;

// const double joint_min[5] = {DEG_TO_RAD(5.0), DEG_TO_RAD(5.0), DEG_TO_RAD(-151.0-146.0+5.0), DEG_TO_RAD(5.0), DEG_TO_RAD(5.0)};
// const double joint_max[5] = {DEG_TO_RAD(169.0+169.0-10.0), DEG_TO_RAD(90.0+65.0-10.0), DEG_TO_RAD(-10.0), DEG_TO_RAD(102.5+102.5-10.0), DEG_TO_RAD(167.5+167.5-10.0)};

void stop_base()
{
  control_mode.data = "Velocity";
  geometry_msgs::Twist command;
  

  // set all velocities to zero
  command.linear.x = 0.0;
  command.linear.y = 0.0;
  command.angular.z = 0.0;

  // send stop command
  base_control_mode_pub.publish(control_mode);
  velocity_command_pub.publish(command);
  
  STOPPED_BASE = true;
}

void stop_arm()
{	
  control_mode.data = "Velocity";
  // set all velocities to zero
  for(int j=0; j<nr_of_joints; j++){
    jvel_msg.velocities[j]=0.0;
  }
  // send stop command
  arm_control_mode_pub.publish(control_mode);
  joint_velocity_pub.publish(jvel_msg);
  STOPPED_ARM = true;
}

int current_joint = 0;
// double joint_states[5];
// void jointStatesCallback(const sensor_msgs::JointState::ConstPtr& msg)
// {
// 	if (!joints_initialized)
// 	{
// 		joints_initialized = true;
// 		for (int i=0; i<5; i++)
// 		{
// 			joint_states[i] = msg->position[i];
// 		}
// 	}
// }

void joystickCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::Twist command;
  control_mode.data = "Velocity";
  if (msg->buttons[1] == true)
  {
    active_control_flag = true;

    // activated left/right turn?
    if (!left_turn_activated && msg->axes[2] != 0)
      left_turn_activated = true;
    if (!right_turn_activated && msg->axes[5] != 0)
      right_turn_activated = true;


    // reset watchdog
    last_signal = ros::Time::now();

    // linear gains
    command.linear.x = linear_velocity_gain * msg->axes[1];
    command.linear.y = linear_velocity_gain * msg->axes[0];

    // angular gain
    double left_turn = 0.0;
    double right_turn = 0.0;
    if (left_turn_activated)
      left_turn = 0.5 * (-msg->axes[2] + 1.0);
    if (right_turn_activated)
      right_turn = 0.5 * (-msg->axes[5] + 1.0);

    const double base_threshold = 1e-3;
    if (left_turn > base_threshold && right_turn < base_threshold)
    {
      command.angular.z = angular_velocity_gain * left_turn;
    }
    else if (right_turn > base_threshold && left_turn < base_threshold)
    {
      command.angular.z = -angular_velocity_gain * right_turn;
    }
    else
    {
      command.angular.z = 0.0;
    }

    if (quadratic_control)
    {
      if (command.linear.x > 0.0)
        command.linear.x = command.linear.x * command.linear.x;
      else
        command.linear.x = - command.linear.x * command.linear.x;

      if (command.linear.y > 0.0)
        command.linear.y = command.linear.y * command.linear.y;
      else
        command.linear.y = - command.linear.y * command.linear.y;

      if (command.angular.z > 0.0)
        command.angular.z = command.angular.z * command.angular.z;
      else
        command.angular.z = - command.angular.z * command.angular.z;
    }

    // send command
    if (fabs(command.linear.x) > base_threshold || fabs(command.linear.y) > base_threshold || fabs(command.angular.z) > base_threshold) {
      base_control_mode_pub.publish(control_mode);
      velocity_command_pub.publish(command);
      STOPPED_BASE = false;
    } else if (!STOPPED_BASE) {
      stop_base();
    }

    // commands for arm
    if (msg->axes[7] <= 0.5 && msg->axes[7] >= -0.5)
      joint_selection_button_released = true;
    if (msg->axes[7] > 0.5 && joint_selection_button_released)
    {
      current_joint = std::min(4, current_joint+1);
      joint_selection_button_released = false;
    }
    if (msg->axes[7] < -0.5 && joint_selection_button_released)
    {
      current_joint = std::max(0, current_joint-1);
      joint_selection_button_released = false;
    }

    double arm_threshold = 1e-2;
    double joint_velocities[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    joint_velocities[current_joint] = -msg->axes[6] * DEG_TO_RAD(10.0);

    if(fabs(joint_velocities[0]) > arm_threshold || fabs(joint_velocities[1]) > arm_threshold ||
        fabs(joint_velocities[2]) > arm_threshold || fabs(joint_velocities[3]) > arm_threshold || fabs(joint_velocities[4]) > arm_threshold)
    {
      for(int j=0; j<nr_of_joints; j++){
        jvel_msg.velocities[j]=joint_velocities[j];
      }
      arm_control_mode_pub.publish(control_mode);
      joint_velocity_pub.publish(jvel_msg);
      STOPPED_ARM = false;
    } else if (!STOPPED_ARM) {
      stop_arm();
    }

    // commands for gripper
    if (msg->buttons[4])
    {
      // close gripper
      gripper_cmd.data = 0;
      gripper_position_pub.publish(gripper_cmd);
    }
    else if (msg->buttons[5])
    {
      // open gripper
      gripper_cmd.data = 1;
      gripper_position_pub.publish(gripper_cmd);
    }
  }
  else
  {
    // button released? send stop to base
    if (active_control_flag)
    {
      active_control_flag = false;
      if (!STOPPED_BASE)
        stop_base();
      if (!STOPPED_ARM)
        stop_arm();
    }
  }
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "joystick_controller");

  ros::NodeHandle nh;
  ros::Rate loop_rate(50);

  
  jvel_msg.velocities.assign(nr_of_joints,0);

  left_turn_activated = false;
  right_turn_activated = false;
  active_control_flag = false;
  STOPPED_BASE = false;
  STOPPED_ARM = false;
  joint_selection_button_released = true;

  // controller gains
  nh.param("linear_velocity_gain", linear_velocity_gain, 1.0);
  nh.param("angular_velocity_gain", angular_velocity_gain, 1.0);

  nh.param("quadratic_control", quadratic_control, true);

  // maximum time of no signal before stopping youbot
  double max_signal_loss_time;
  nh.param("max_signal_loss_time", max_signal_loss_time, 0.5);

  // subcriber and publisher
  ros::Subscriber joystick_sub = nh.subscribe("/joy", 1, joystickCallback);
  //ros::Subscriber joint_states_sub = nh.subscribe("/joint_states", 1, jointStatesCallback);

  velocity_command_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  gripper_position_pub = nh.advertise<std_msgs::Int32>("/arm_1/gripper_controller/position_command", 1);
  //joint_position_pub = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command", 1);
  joint_velocity_pub = nh.advertise<motion_control_msgs::JointVelocities>("/arm_1/arm_controller/velocity_command", 1);
  arm_control_mode_pub = nh.advertise<std_msgs::String>("/arm_control_mode",1);
  base_control_mode_pub = nh.advertise<std_msgs::String>("/base_control_mode",1);
  // main loop
  while (ros::ok())
  {
    // watchdog: if no message for too long, stop youbot!
    if (ros::Time::now() - last_signal > ros::Duration(max_signal_loss_time) && active_control_flag)
    {
      ROS_WARN("Stopping youBot since no signal from remote for too long.");
      if (!STOPPED_BASE)
        stop_base();
      if (!STOPPED_ARM)
        stop_arm();
    }

    // wait
    ros::spinOnce();
    loop_rate.sleep();
  }
  //control_mode.data = "MotorStop";
  //base_control_mode_pub.publish(control_mode);	
  //arm_control_mode_pub.publish(control_mode);	
  return 0;
}
