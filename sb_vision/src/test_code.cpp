//-------------------------------------------------
// UBC Snowbots
// test_code.cpp
//        Purpose: To Test Code
// Author: Otto


#include "ros/ros.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace ros;
using namespace std;
using namespace cv;


static const string NODE_NAME ="test_code";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
	Mat image;
	image = imread("Banana.jpg", CV_LOAD_IMAGE_COLOR);

	namedWindow( "Banana", WINDOW_AUTOSIZE );// Create a window for display.
    	//imshow( "Banana", image );                   // Show our image inside it.

    	waitKey(0);                                // Wait for a keystroke in the window
    
	init(argc, argv, NODE_NAME); //initializes your ROS node
     while (ros::ok())
    {
    ROS_INFO("Hi!"); // Publishes to log and terminal
    // Put all your other code in here
    }
return 0;
}
//-------------------------------------------------
