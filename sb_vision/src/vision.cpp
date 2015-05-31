// UBC Snowbots
// vision.cpp
//		Purpose: vision node to combine image stitcher, filtering, and bird's eye view transformation
//				
// Author: Jannicke Pearkes


#include "ros/ros.h" 

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "filter.h"
#include "IPM.h"

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
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("stitcher", 1);
    sensor_msgs::ImagePtr msg;
	 
    Mat inputImg, inputImgGray;
	Mat outputImg;
    Mat frame;
    Mat image2;
        
    //IPM Set-up, will put into class itself soon
    int width = 1920;
    int height = 1080;

    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height) );
    origPoints.push_back( Point2f(width, height) );
    //origPoints.push_back( Point2f(width/2+450, 580) );
    //origPoints.push_back( Point2f(width/2-250, 580) );
    origPoints.push_back( Point2f(width/2+450, 620) );
    origPoints.push_back( Point2f(width/2-250, 620) );

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );
		

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
	    
        //Currently using static image for debugging purposes, seem to need an absolute
        // path to the image to avoid crashes
    	cv::Mat image = imread("/home/jannicke/Pictures/Image1.jpg", 1);

        if(!image.data) std::cout<<"NO IMAGE DATA"<<std::endl;
        else std::cout<<"image data exists"<<std::endl;
        std::cout<<"image read"<<std::endl;
        //TODO: add in image stitching here

        filter myfilter; //create filter
        image2 = myfilter.getUpdate(image);
        myfilter.displayImages(image);
        
        //TODO: add in transform
	    // IPM object
	    IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );
        //TODO: add publish the output image

		// Get current image
		//cap.read(frame);	

		//frame = imread("/home/jannicke/Pictures/Image2.jpg", 1);
		frame = image2;

		if( frame.empty() ) break;

		// Color Conversion
		if(frame.channels() == 3)cvtColor(frame, inputImgGray, CV_BGR2GRAY);				 		 
		else frame.copyTo(inputImgGray);			 		 

		// Process
		ipm.applyHomography( frame, outputImg );		 
		ipm.drawPoints(origPoints, frame );
        
        //TODO: Output image 
        if(!outputImg.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", outputImg).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }
	    //ros::Publisher image_pub = nh.advertise<std_msgs:: image>(PUBLISH_TOPIC);
        ROS_INFO("Vision published an image");
        ros::spinOnce();
        loop_rate.sleep();

    }

}
    

   