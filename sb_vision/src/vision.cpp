// UBC Snowbots
// vision.cpp
//		Purpose: vision node to combine image stitcher, filtering, and bird's eye view transformation
//				
// Author: Jannicke Pearkes


#include "ros/ros.h" 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "filter.h"
// Namespaces
using namespace cv;
using namespace ros; 
using namespace std; 

static const string VISION_NODE_NAME = "vision";
//static const string PUBLISH_TOPIC = "image";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
{
	ros::init(argc,argv, VISION_NODE_NAME);
	ros::NodeHandle n;
	ros::Rate loop_rate(10);

	while(ros::ok())
    {
        // Read in image from video camera, or other source
    	 /*
        cap >> image; //from video
 
	    if (!cap.read(image)) //if not success, break loop
	    {
	        cout << "Cannot read a frame from video stream" << endl;
	        break;
	    }*/
	    
        //Currently using static image for debugging purposes
    	cv::Mat image = imread("/home/jannicke/Pictures/Image1.jpg", 1);
        if(!image.data) std::cout<<"NO IMAGE DATA"<<std::endl;
        else std::cout<<"image data exists"<<std::endl;
        std::cout<<"image read"<<std::endl;
        //TODO: add in image stitching here

        filter myfilter; //create filter
        image = myfilter.getUpdate(image);
        myfilter.displayImages(image);
        
        //TODO: add in transform
        
        //TODO: add publish the output image

	    //ros::Publisher image_pub = nh.advertise<std_msgs:: image>(PUBLISH_TOPIC);
        ROS_INFO("Vision published an image");
        ros::spinOnce();
        loop_rate.sleep();

    }

}
    

   