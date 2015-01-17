//-------------------------------------------------
// UBC Snowbots
// Tutorialcode.cpp
//        Purpose: To test a code
// Author: Calvin Xu


#include "ros/ros.h"

using namespace ros;
using namespace std;

static const string NODE_NAME = "Tutorialcode";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
    init(argc, argv, NODE_NAME); //initializes your ROS node
     while (ros::ok())
    {
    ROS_INFO("Hello Everyonee!"); // Publishes to log and terminal
    // Put all your other code in here
    }
return 0;
}
//-------------------------------------------------

