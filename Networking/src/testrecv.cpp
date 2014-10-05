/**
 * UBC SnowBots
 * 
 * testrecv.cpp
 * 	this simple test receiver listens on "chatter" and prints any data it receives
 * 
 * */

#include "ros/ros.h"
#include "test/Test.h"

void printCallback(const test::Test::ConstPtr& msg){
	ROS_INFO("[%s] received", msg->test.c_str());
}

int main(int argc, char** argv){
	ros::init(argc,argv,"listener");
	ros::NodeHandle n;
	
	ros::Subscriber sub = n.subscribe("chatter", 1000, printCallback);
	ros::spin();
	
	return 0;
}
