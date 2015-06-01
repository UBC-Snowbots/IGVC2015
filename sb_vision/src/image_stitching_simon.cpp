#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream> 
#include <vector>
#include <signal.h>

using namespace cv;

static const std::string NODE_NAME = "ImageStitcher";
static const std::string CVWINDOW = "Sticher Window";
static const unsigned int CAMERA_AMOUNT = 3;

//Putting these inside an array seems to cause the program to not update...why?
VideoCapture cap1, cap2, cap3;

/*
This function handles the releasing of objects when this node is
requested or forced (via CTRL+C) to shutdown.
*/
void onShutdown(int sig){
	destroyWindow(CVWINDOW);
	cap1.release();
	cap2.release();
	cap3.release();
	ROS_INFO("All objects should have been released, proper shutdown complete");
	ros::shutdown();
}

/*
This function intialize the connection to each webcam
	
The deviceID for all three webcams will be between [1,3]
Once a connection is established, the connected camera will occupied 
the lowest deviceID avalible, making the next connection ID predictable
*/
bool connectToCamera(VideoCapture& camera){
	static unsigned int occupiedID = 1;
	
	camera.set(CV_CAP_PROP_FPS, 15);
	
	//Our webcams support 1080p recording, making the ideal resolution 1920x1080
	//However this large resolution will significantly impact the sticher's performance
	//Therefore we need to determine an ideal resolution
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 960);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 540);

	for (int deviceID = occupiedID; deviceID <= CAMERA_AMOUNT; deviceID++){
		if (camera.open(deviceID)){
			ROS_INFO("Successfully established conntection to webcam %d", deviceID);
			occupiedID++;
			return true;
		}
	}

	ROS_FATAL("Unable to establish connection to webcam %d", occupiedID);
	return false;
}


int main(int argc, char **argv)	{
	ros::init(argc, argv, NODE_NAME, ros::init_options::NoSigintHandler);
	signal(SIGINT, onShutdown);
	
	if(!connectToCamera(cap1) || !connectToCamera(cap2) || !connectToCamera(cap3)){
		ROS_FATAL("Unable to connect to all cameras, exiting now");
		ROS_FATAL("If this error persist, run the usb-reset.sh in the home directory");
		return 0;
	}
	
	Mat image1, image2, image3;		
	std::vector<Mat> imgs;
	Stitcher stitcher = Stitcher::createDefault(true);

	int counter = 0;
	int errorCounter = 0;
	bool retry = false;
	namedWindow(CVWINDOW);	
	//ros::Duration(1).sleep();	//Give some time for the camera to warm up

	while (ros::ok() && errorCounter < 5){
		ROS_INFO("Image Stitching Started!");
		counter++;
		
		/*
		When reading from the camera, there is a chance of an error occuring.
		However these aren't OpenCV errors, so a try/catch block is useless :(

		This error will produce the following messages on the terminal
			libv4l2: error queuing buf 0: No such device
			libv4l2: error queuing buf 1: No such device
			libv4l2: error dequeuing buf: No such device
			VIDIOC_DQBUF: No such device
		and in the next iteration, the following error messages is printed before crashing
			libv4l2: error dequeuing buf: No such device
			VIDIOC_DQBUF: No such device
			Segmentation fault(core dumped)
		*/
			
		if (!cap1.read(image1)){
			ROS_WARN("Cannot read image 1 from video stream");
			errorCounter++;
			continue;
		}
		if (!cap2.read(image2)){
			ROS_WARN("Cannot read image 2 from video stream");
			errorCounter++;
			continue;
		}
		if (!cap3.read(image3)){
			ROS_WARN("Cannot read image 3 from video stream");
			errorCounter++;
			continue;
		}
	
		if (image1.empty() || image2.empty() || image3.empty())
			ROS_WARN("One of the Mat is empty");

		//Delcaring the @pano inside the loop seems to significantly decrease
		//the chance of an error happening. Perhaps there's some old data that
		//wasn't properly destroyed...
		Mat pano;
		imgs.push_back(image1);
		imgs.push_back(image2);
		imgs.push_back(image3);

		Stitcher::Status status= stitcher.stitch(imgs, pano);
		imgs.clear();
		
		if (status != Stitcher::OK) {
			ROS_FATAL("Unable to stitch images together!, trying again...");
			continue;

		} else {			    
			ROS_INFO("Awaiting for stiched image to display");
			imshow(CVWINDOW, pano);
			if(waitKey(25) == 27){
				ROS_INFO("ESC key pressed! Exiting loop now");
				ROS_WARN("The next run has a higher chance of crashing for unknown reasons");
				break;
	       	}
		}
		ROS_INFO("Iteration: %d", counter);
	}

	onShutdown(0);	
	return 0;
}
