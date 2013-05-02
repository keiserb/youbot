#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "brics_actuator/JointPositions.h"
#include "sensor_msgs/JointState.h"
#include "rpg_youbot_common.h"
#include <iostream>

using namespace std;
geometry_msgs::Twist command;			//define message which controls base
brics_actuator::JointPositions jointpos;
ros::Publisher velocity_command_pub;	//define publisher	
ros::Publisher joint_command_pub;
ros::Subscriber joint_state;
 	

void argparser(char key, double * joints)
{
		if(key=='w')
		{
			command.linear.x = command.linear.x+0.1;
		}
		if(key=='s')
		{
			command.linear.x = command.linear.x-0.1;
		}
		if(key=='e')
		{
			command.linear.y = command.linear.y+0.1;
		}
		if(key=='d')
		{
			command.linear.y = command.linear.y-0.1;
		}
		if(key=='r')
		{
			command.linear.z = command.linear.z+0.1;
		}
		if(key=='f')
		{
			command.linear.z = command.linear.z-0.1;
		}
		if(key=='t')
		{
			command.angular.x = command.angular.x+0.1;
		}
		if(key=='g')
		{
			command.angular.x = command.angular.x-0.1;
		}
		if(key=='z')
		{
			command.angular.y = command.angular.y+0.1;
		}
		if(key=='h')
		{
			command.angular.y = command.angular.y-0.1;
		}
		if(key=='u')
		{
			command.angular.z = command.angular.z+0.1;
		}
		if(key=='j')
		{
			command.angular.z = command.angular.z-0.1;
		}
		if(key=='i')
		{
			joints[0]=joints[0]+0.1;
		}
		if(key=='k')
		{
			joints[0]=joints[0]-0.1;
		}
		if(key=='o')
		{
			joints[1]=joints[1]+0.1;
		}
		if(key=='l')
		{
			joints[1]=joints[1]-0.1;
		}
		if(key=='y')
		{
			joints[2]=joints[2]+0.1;
		}
		if(key=='x')
		{
			joints[2]=joints[2]-0.1;
		}
		if(key=='c')
		{
			joints[3]=joints[3]+0.1;
		}
		if(key=='v')
		{
			joints[3]=joints[3]-0.1;
		}
		if(key=='b')
		{
			joints[4]=joints[4]+0.1;
		}
		if(key=='n')
		{
			joints[4]=joints[4]-0.1;
		}

}

void printmsg(const sensor_msgs::JointState::ConstPtr& msg)
{
	cout << "message: " << endl;
	for(int i=0; i<5; i++)
	{
		cout << "name: " << msg->name[i] << endl;
		cout << "position: " << msg->position[i] << endl;
		cout << "velocity: " << msg->velocity[i] << endl;
		//cout << "effort: " << msg->effort[i] << endl;
	} 

}

int main(int argc, char* argv[])
{
	char key = 'a';
	ros::init(argc, argv, "control");
	ros::NodeHandle nh;
	ros::Rate loop_rate(10);
	//advertise(): Topic we want to publish under + buffer size
	velocity_command_pub = nh.advertise<geometry_msgs::Twist>("twist_in", 1);
	joint_command_pub = nh.advertise<brics_actuator::JointPositions>("jpos_in",1);
	
	//set desired joint positions and generate message
	double joints[5] = { 0.0, 0.0, 0.0, 0.0, 0.0 }; 
	jointpos = rpg_youbot_common::generate_joint_position_msg(joints);

	//set desired speeds
	command.linear.x = 0.0;
	command.linear.y = 0.0;
	command.linear.z = 0.0;
	command.angular.x = 0.0;
	command.angular.y = 0.0;
	command.angular.z = 0.0;

	
	while(key!='q')
	{
		cout<< "enter command: " << endl;
		cin >> key;
		
		argparser(key,joints);
		jointpos = rpg_youbot_common::generate_joint_position_msg(joints);

		// publish messages
		velocity_command_pub.publish(command);
		joint_command_pub.publish(jointpos);

		ros::spinOnce(); 
		
 		// wait
		joint_state = nh.subscribe("jstate_out",1,printmsg); 		
 		loop_rate.sleep();	

	}
	

	return 0;
}

