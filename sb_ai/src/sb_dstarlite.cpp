#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"


// callback to receive data from subscription
void map_callback(const std_msgs::String::ConstPtr& msg)
{

}

// main function that runs this node
int main(int argc, char **argv)
{

	// Initializes this node. 
	// First 2 arguments are for ROS arguments and name remapping.
	// Last argument is for the name of this node.
	ros::init(argc, argv, AI_NODE_NAME);
	
	// Main access point to communication with ROS system.
	ros::NodeHandle ai_nh;
	
	// Initialize publisher to PUB_TOPIC.
	// Second argument for advertise is the buffer size.
	ros::Publisher car_pub = nh.advertise<std_msgs::String>(PUB_TOPIC, 100);
	
	// Frequency of the loop. 
	// eg. 10 = 10hz
	ros::Rate loop_rate(10);
	
	int count = 0;
	
	while (ros::ok()) {
		
		std_msgs::String msg;
		
		std::stringstream ss;
		ss << "hello world " << count;
		
		msg.data = ss.str();
		
		car_pub.publish(msg);
		
		ros::spinOnce();	// required for subscriptions
		
		loop_rate.sleep();	// sleep for the time remaining to hit frequency loop rate
		++count;
		
	}
	return 0;
}
