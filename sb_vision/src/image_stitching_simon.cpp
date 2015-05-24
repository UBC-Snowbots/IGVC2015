#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream> 
#include <vector>

using namespace ros;
using namespace cv; 
using namespace std;

static const string NODE_NAME = "descriptive_name";
const int MSG_QUEUE_SIZE = 20;

bool connectCamera(VideoCapture& camera){
	//TODO: Is there a way to tell which port the webcams auto-connect to?
	ROS_INFO("Init webcam");
	camera.set(CV_CAP_PROP_FPS,15);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	for (int portNumber = 10; portNumber >= 0; portNumber--){
		ROS_INFO("Attempting port: %d", portNumber);
		if (camera.open(portNumber)){
			ROS_INFO("Connection established on port: %d", portNumber);
			return true;
		}
	}

	ROS_INFO("Unable to establish connection on ports");
	
	return false;
}

int main(int argc, char **argv)	
{
	init(argc, argv, NODE_NAME);

	VideoCapture cap1;
	//VideoCapture cap2;
	//VideoCapture cap3;
									
	if(!connectCamera(cap1)/* || !connectCamera(cap2) || !connectCamera(cap3)*/){
		ROS_INFO("Unable to connect to all cameras, exiting now");
		return 0;
	}
							
	Mat image1, image2;// image3;
	Stitcher stitcher = Stitcher::createDefault();

	while (ros::ok()){
		ROS_INFO("Image Stitching Started!");

		if (!cap1.read(image1)){
			ROS_INFO("Cannot read image 1 from video stream");
			//continue;
		}				
		/*	 
		if (!cap2.read(image2)){
			ROS_INFO("Cannot read image 2 from video stream");
			//continue;
		}
			
		if (!cap3.read(image3)){
			ROS_INFO("Cannot read image 3 from video stream");
			//continue;
		}
		*/
		Mat pano;
		vector<Mat> imgs;
		/*
			What is .data() doing?, what about using .empty() to
			determine if the matrix is empty or not
		*/	
		if (image1.empty() /*|| image2.empty() || image3.empty()*/){
			ROS_INFO("One of the Mat is empty");
			//continue; 
		}

		imgs.push_back(image1);
		//imgs.push_back(image2);
		//imgs.push_back(image3);

		Stitcher::Status status = stitcher.stitch(imgs, pano);
			
		imgs.pop_back();
		//imgs.pop_back();
		//imgs.pop_back();
			
		if (status != Stitcher::OK) {
			ROS_INFO("Error stitching");
			break;
		} else {			    
			//imshow("Video 2", image2);
			ROS_INFO("Awaiting image display");
			imshow("Stitched Video", pano);
			ROS_INFO("Displayed");
		}

		char input = waitKey(1000);
		if(input == 'q')
			break;
	}
	
	ROS_INFO("Releasing VideoCapture objects!");
	cap1.release();
	//cap2.release();
	//cap3.release();
	ROS_INFO("Goodbye");

	return 0;
	
}
