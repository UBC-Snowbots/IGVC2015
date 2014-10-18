/**
 * UBC SnowBots
 * 
 * testrecv.cpp
 * 	this simple test receiver listens on "chatter" and prints any data it receives
 * 
 * */

#include "ros/ros.h"
#include "std_msgs/String.h"

void printCallback(const std_msgs::String::ConstPtr& msg){
	ROS_INFO("[%s] received", msg->data.c_str());
}

int main(int argc, char** argv){
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("chatter", 1000, printCallback);
	ros::spin();
	
	return 0;
}
