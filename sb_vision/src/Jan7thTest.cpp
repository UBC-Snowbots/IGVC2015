//-------------------------------------------------
// UBC Snowbots
// Jan7thTest.cpp
//        Purpose: To test cpp running ability with rosrun
// Author: ziyue(Grace) Hu


#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace ros;
using namespace std;
using namespace cv;

static const string NODE_NAME = "runThis";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
    Mat image;
    image = imread("fishImage.jpg");
    namedWindow( "fishImage", WINDOW_AUTOSIZE );
    // Create a window for display.
    //imshow( "fishImage", image );                   
    // Show our image inside it.*Names HAVE to be the same
    //waitKey(0); // Wait for a keystroke in the window
    init(argc, argv, NODE_NAME); //initializes your ROS node
     while (ros::ok())
    {
    ROS_INFO("Hi!"); // Publishes to log and terminal
    // Put all your other code in here
    }
return 0;
}
