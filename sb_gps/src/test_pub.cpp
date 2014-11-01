#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;
using namespace ros;

static const string NODE_NAME = "sample_gps";
static const string GPS_DATA_TOPIC = "sample_coord";
double curr_lat = 200, curr_long = 190; // keep adding 2 or something every second or so

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME);
	NodeHandle n;
	Publisher gps_data_pub = n.advertise<std_msgs::String>(GPS_DATA_TOPIC, 20);
	Rate loop_rate(1); // one cycle per sec ?
	
	while (ok())
	{
		std_msgs::String msg;
		msg.data = "B,4.267821e+08,-8.319514e+09,30.33";		
		ROS_INFO("%s",msg.data.c_str());
		gps_data_pub.publish(msg);
		spinOnce();
		loop_rate.sleep();
	}
	return 0;
}



