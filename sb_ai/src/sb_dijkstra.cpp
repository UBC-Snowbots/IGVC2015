#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include "sb_msgs/AISimMap.h"
#include "AI/dijkstra/dijkstra.h"
#include "AI/dijkstra/dijkstra.cpp"	// need to create makefile
#include <geometry_msgs/Twist.h>
#include <vector>

using namespace std;

int * map_ptr;
std::vector<int> map;
int width, height;
int START = 10;
int GOAL = 98;
geometry_msgs::Twist twist_msg;
Dijkstra dijkstras;

// callback to receive data from subscription
// this is where you do what you need to do with the map data
void map_callback(const sb_msgs::AISimMap::ConstPtr& msg)
{
	//map_ptr = msg->map_grid;
	width = msg->width;
	height = msg->height;
	map_ptr = new int[width*height];

	for (int i = 0; i < width*height; i++) {
		map_ptr[i] = msg->map_grid[i];
	}

	//ROS_INFO("I heard: [%s]", msg->data.c_str());
}


// main function that runs this node
int main(int argc, char **argv)
{

	// Initializes this node. 
	// First 2 arguments are for ROS arguments and name remapping.
	// Last argument is for the name of this node.
	ros::init(argc, argv, AI_NODE_NAME);
	
	// Main access point to communication with ROS system.
	ros::NodeHandle nh;
	
	// Initialize publisher to PUB_TOPIC.
	// Second argument for advertise is the buffer size.
	ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 100);	
	ros::Subscriber map_sub = nh.subscribe(MAP_SUB_TOPIC, 100, map_callback);
	
	// Frequency of the loop. 
	// eg. 10 = 10hz
	ros::Rate loop_rate(10);


	
	while (ros::ok()) {
		
		dijkstras.Init(map_ptr, width, height, START, GOAL);
		dijkstras.Search(dijkstras.GetStart());
		dijkstras.ReconstructPath(dijkstras.GetGoal());

		twist_msg.linear.x = 0;
		twist_msg.linear.y = 0;
		twist_msg.linear.z = 0;
		
		twist_msg.angular.x = 0;
		twist_msg.angular.y = 0;
		twist_msg.angular.z = 0;

		car_pub.publish(twist_msg);	// this is where we publish our map analysis
		
		ros::spinOnce();	// required for subscriptions
		
		loop_rate.sleep();	// sleep for the time remaining to hit frequency loop rate
	
	}

	return 0;
}
