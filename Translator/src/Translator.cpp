#include "Translator.hpp"



using namespace std;

void printmsg(const sensor_msgs::JointState::ConstPtr& msg)
{
        cout << "message: " << endl;
        for(int i=0; i<5; i++)
        {
                cout << "name: " << msg->name[i] << endl;
                cout << "position: " << msg->position[i] << endl;
                cout << "velocity: " << msg->velocity[i] << endl;
        }

}

int main(int argc, char* argv[])
{
        char key = 'a';
        ros::init(argc, argv, "control");
        ros::NodeHandle nh;
        ros::Rate loop_rate(200);
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
