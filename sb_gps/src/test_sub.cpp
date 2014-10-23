#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sstream>
#include <iostream>
#include <string>
#include <math.h>

using namespace std;

static const string NODE_NAME = "subscriber";
static const string GPS_OUTPUT_TOPIC = "temp_output";

void gpsCallback(const geometry_msgs::Twist::ConstPtr& msg) 
{
	ROS_INFO("Got back: [Lin.x = %f, Lin.y = %f, Ang.z = %f]", msg->linear.x, msg->linear.y, msg->angular.z);
}

int main(int argc, char **argv) 
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe(GPS_OUTPUT_TOPIC, 20, gpsCallback);
	ros::spin();
	return 0;
}
