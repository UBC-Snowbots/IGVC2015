#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream> 
#include <vector>

using namespace cv;

static const string NODE_NAME = "ImageStitcher";
static const unsigned int CAMERA_AMOUNT = 3;

/*
This function intialize the connection to each webcam
	
The deviceID for all three webcams will be between [1,3]
Once a connection is established, the connected camera will occupied 
the lowest deviceID avalible, making the next connection ID predictable
*/
bool connectToCamera(VideoCapture& camera){
	static unsigned int occupiedID = 1;
	
	camera.set(CV_CAP_PROP_FPS, 20);
	
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

	ROS_FATAL("Unable to establish connection to webcam %d, exiting now", occupiedID);
	return false;
}


int main(int argc, char **argv)	{
	ros::init(argc, argv, NODE_NAME);

	VideoCapture cameras[CAMERA_AMOUNT];
	Mat images[CAMERA_AMOUNT];
	std::vector<Mat> imgs;
	Stitcher stitcher = Stitcher::createDefault(true);

	for (int i = 0; i < CAMERA_AMOUNT; ++i){
		if(!connectToCamera(cameras[i])){
			ROS_FATAL("If this error persist, run the usb-reset.sh and retry");
			return 0;
		}
	}

	int counter = 0;
	int errorCounter = 0;
	bool retry = false;
	namedWindow("Stiching Window");	

	while (ros::ok() && counter < 5 && errorCounter < 5){
		//Testing to see if having the mat obj recreated every loop would
		//decrease the chances of encountering the error again...
		

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
	
		//According to the OpenCV documentation: >> does the same as .read()
		//However from testing the code, if I attempt to read without doing >> first, 
		//the initial run of this program will always fail, while all subsequent ones run fine...
		for (int i = 0; i < CAMERA_AMOUNT; ++i){
			cameras[i] >> images[i];
			if (!cameras[i].read(images[i]) || images[i].empty()){
				ROS_ERROR("Failed to read proper image from camera %d", i);
				errorCounter++;
				retry = true;
				break;
			}
			imgs.push_back(images[i]);
		}

		if(retry){
			ROS_ERROR("Images incomplete, trying again...");
			imgs.clear();
			retry = false;
			continue;
		}


		//For some reason, declaring the matrix which will hold
		//the stitched image INSIDE the loop significantly reduces
		//the chance of the error happening. Perhaps the Mat needs to
		//be cleared or re-created for better quality
		Mat pano;

		Stitcher::Status status = stitcher.stitch(imgs, pano);
		imgs.clear();
		
		if (status != Stitcher::OK) {
			ROS_FATAL("Unable to stitch images together!, trying again...");
			continue;

		} else {			    
			ROS_INFO("Awaiting for stiched image to display");
			/*
			At this point the stiched image is ready in the Mat object
			If you want to process the Mat directly without displaying
			the image then there's no problem.

			HOWEVER... if you need to display the image continously				
			things will start to go wrong
				
			We are using ros::ok to continously run this program
			and imshow() displays the stitched image to the window
			Therefore closing the window via the 'x' button will 
			not stop the program since we're in a loop
				
			Only ctrl+c will stop the loop, but this is terrible because the code
			is forced to quit and it will never be able to release the webcams
			with .release() - This causes the webcam to not connect the next time
			this program is executed, without having to manually re-connected it again
			*/
			imshow("Stiching Window", pano);
			if(waitKey(50) == 27){
				ROS_INFO("ESC key pressed! Exiting loop now");
				ROS_WARN("The next run has a high chance of crashing for unknown reasons");
				break;
	       	}
			destroyWindow("Stiching Window");
			//ROS_INFO("Destroyed stitch image window");
		}
		ROS_INFO("Counter: %d", counter);
	}
	
	//If you CTRL + C while inside the ros::ok loop, this part will never get executed
	//causing a re-connection problem to the webcams in any subsequent execution.
	
	ROS_INFO("Releasing VideoCapture objects!");
	for (int i = 0; i < CAMERA_AMOUNT; ++i){
		cameras[i].release();
	}
	ROS_INFO("All VideoCaptures should have been released, proper shutdown complete");
	
	return 0;
}
