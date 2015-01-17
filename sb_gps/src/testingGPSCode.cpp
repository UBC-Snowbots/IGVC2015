//-------------------------------------------------
// UBC Snowbots
// testingGPSCode.cpp
//   	 Purpose: Testing out a code
// Author: Nichlas Wu


#include "ros/ros.h"

using namespace ros;
using namespace std;

static const string NODE_NAME = "testingGPSCode";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
//Stuff to run once 
    init(argc, argv, NODE_NAME); //initializes your ROS node
	 while (ros::ok())
	{
	ROS_INFO("Hi!"); // Publishes to log and terminal
	// Put all your other code in here
	}
return 0;
}
//-------------------------------------------------

