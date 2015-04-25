#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <iostream>
#include "sb_ai.h"
#include "sb_msgs/AISimMap.h"
#include "sb_msgs/AISimGPS.h"
#include "AI/dijkstra/dijkstra.h"
#include "AI/dijkstra/dijkstra.cpp"	// TODO need to create makefile
#include <geometry_msgs/Twist.h>

using namespace std;

int * map_ptr;
int width = 0; 
int height = 0;
int start = 90;
int goal = 11;
geometry_msgs::Twist twist_msg;
Dijkstra dijkstras;
int next_movement = 0;

/** Callback functions that receive data from subscriptions and process them */

// The gps waypoint data is processed here
void gps_callback(const sb_msgs::AISimGPS::ConstPtr& msg) 
{
	start = (msg->latitude * width) + msg->longitude;
	ROS_INFO("Lat: %ld", msg->latitude);
	ROS_INFO("Long: %ld", msg->longitude);
	ROS_INFO("Theta: %ld", msg->theta);
}


// The map data is processed here
void map_callback(const sb_msgs::AISimMap::ConstPtr& msg)
{
	free(map_ptr);	// free memory allocated by map previously
	width = msg->width;
	height = msg->height;
	map_ptr = new int[width*height];

	// copy map contents
	for (int i = 0; i < width*height; i++) {
		map_ptr[i] = msg->map_grid[i];
	}
}


// calculate twist message for next move
geometry_msgs::Twist GetTwistMsg(int next_move) 
{
	geometry_msgs::Twist twist;

	twist.linear.x = 0;
	twist.linear.y = 0;
	twist.linear.z = 0;
		
	twist.angular.x = 0;
	twist.angular.y = 0;
	twist.angular.z = 0;

	if (next_move == -1 || next_move == 1) { twist.linear.x = next_move*0.2; }
	if (next_move == -2 || next_move == 2) { twist.linear.y = next_move*0.1; }
	return twist;
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
	ros::Publisher car_pub = nh.advertise<geometry_msgs::Twist>(PUB_TOPIC, 1);	
	ros::Subscriber map_sub = nh.subscribe(MAP_SUB_TOPIC, 1, map_callback);
	ros::Subscriber gps_sub = nh.subscribe(GPS_SUB_TOPIC, 1, gps_callback);
	
	// Frequency of the loop. 
	// eg. 10 = 10hz
	ros::Rate loop_rate(2);

	while (ros::ok()) {
		
		if (dijkstras.Init(map_ptr, width, height, start, goal)) {
			dijkstras.Search(dijkstras.GetStart());
			dijkstras.ReconstructPath(dijkstras.GetGoal());
			cout << endl;
			next_movement = dijkstras.GetNextStep();
		}

		twist_msg = GetTwistMsg(next_movement);

		car_pub.publish(twist_msg);	// this is where we publish our map analysis
		
		cout << "Linear x: " << twist_msg.linear.x << endl;
		cout << "Linear y: " << twist_msg.linear.y << endl;
		cout << "Current position: " << start << endl;

		ros::spinOnce();	// required for subscriptions
		
		loop_rate.sleep();	// sleep for the time remaining to hit frequency loop rate
	
	}

	return 0;
}
