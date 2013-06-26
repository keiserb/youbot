#include <iostream>
#include <time.h>
#include <ros/ros.h>

using namespace std;

timespec diff(timespec start, timespec end)
{
  timespec temp;
  if ((end.tv_nsec - start.tv_nsec) < 0)
  {
    temp.tv_sec = end.tv_sec - start.tv_sec - 1;
    temp.tv_nsec = 1000000000 + end.tv_nsec - start.tv_nsec;
  }
  else
  {
    temp.tv_sec = end.tv_sec - start.tv_sec;
    temp.tv_nsec = end.tv_nsec - start.tv_nsec;
  }
  return temp;
}

int main(int argc, char* argv[])
{
  double tdiff;
  ros::Time start, current;
  ros::Duration rdiff;
  timespec t0, t1;
  ros::init(argc, argv, "timing");
  ros::NodeHandle nh;

  ros::Rate loop_rate(10);
  while(ros::ok())
  {
  	clock_gettime(CLOCK_REALTIME, &t0);
  	start = ros::Time::now();
    loop_rate.sleep();
    clock_gettime(CLOCK_REALTIME, &t1);
    current = ros::Time::now();
    rdiff = current - start;
    tdiff = rdiff.toNSec();
    tdiff = tdiff / 1000000.0;
    cout << "execution time in seconds: " << diff(t0, t1).tv_sec << endl;
    cout << "execution time in milliseconds: " << diff(t0, t1).tv_nsec / 1000000.0 << endl;
    cout << "ROS exec time in milliseconds: " << tdiff << endl;

  }
  return 0;
}