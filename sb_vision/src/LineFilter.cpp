// UBC Snowbots
// LineFilter.cpp
//		Purpose: Takes in an image from the camera and filters
//				the image to isolate white lines on the field
// Author: Jannicke Pearkes


#include "ros/ros.h"

using namespace ros; 
using namespace std; 

static const string NODE_NAME = "LineFilter";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME); //initializes your ROS node
 	while (ros::ok())
    {
    ROS_INFO("Hi!"); // Publishes to log and terminal
    //ros::spinOnce();
    }
return 0;
}