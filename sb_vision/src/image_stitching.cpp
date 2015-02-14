//-------------------------------------------------

// UBC Snowbots

// image_stitching.cpp

// Purpose: To stitch two images taken from two webcams on a robot

// Author: Rishabh Singal

#include "ros/ros.h"

using namespace ros;

using namespace std;

static const string NODE_NAME = "descriptive_name";

const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
	
{

 init(argc, argv, NODE_NAME); //initializes your ROS node

while (ros::ok())

{

ROS_INFO("Hi!"); // Publishes to log and terminal

// Put all your other code in here

}

return 0;

}
