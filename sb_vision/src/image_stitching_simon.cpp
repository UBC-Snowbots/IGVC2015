#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream> 
#include <vector>
#include <iterator>

using namespace ros;
using namespace cv; 

static const unsigned int CAMERA_AMOUNT = 3;
static const string NODE_NAME = "descriptive_name";
const int MSG_QUEUE_SIZE = 20;

//THE FOLLOWING CODE WAS MODIFIED AND STILL UNTESTED AT RUNTIME

bool connectToCamera(VideoCapture& camera){
	//TODO: Is there a way to tell which port the webcams auto-connect to?

	ROS_INFO("Initializing Webcam");
	camera.set(CV_CAP_PROP_FPS, 30);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
    //Is it possible for the device to have greater id?!?
	for (int deviceID = 0; deviceID < 5; deviceID++){
		ROS_INFO("Attempting connection to device: %d", deviceID);
		if (camera.open(deviceID)){
			return true;
		}
	}

	return false;
}

int main(int argc, char **argv)	
{
	init(argc, argv, NODE_NAME);
    
    std::cout << "OpenCV version: " << CV_VERSION_MAJOR << "-" << CV_VERSION_MINOR << std::endl; 

	Stitcher stitcher = Stitcher::createDefault();
	VideoCapture cameras[CAMERA_AMOUNT];

	Mat images[CAMERA_AMOUNT];
	Mat stitchedImages;
	std::vector<Mat> inputImages;
	inputImages.reserve(CAMERA_AMOUNT);

	for(int i = 0; i < CAMERA_AMOUNT; i++){
        if(!connectToCamera(cameras[i])){
            ROS_FATAL("Unable to connect to webcam %d", i);
            ROS_FATAL("If this connection problem persist:\n\
						Please disconnect all devices from the computer and restart :(");
            return -1;
        }else{
			ROS_INFO("Connection established for device: %d", i);
		}
    }
	
	int counter = 0;
	namedWindow("Stiching Window");

	ROS_INFO("Entering ROS loop...");
	while (ros::ok() && counter < 5){
		ROS_INFO("Image Stitching Started!");	
		counter++;
		

		for(int i = 0; i < CAMERA_AMOUNT; i++){
			cameras[i] >> images[i];
			if(!cameras[i].read(images[i])){
				ROS_ERROR("Cannot capture image on camera %d", i);
			}
			
			if(images[i].empty()){
				ROS_ERROR("Capture image on camera %d was empty", i);
			}

			inputImages.push_back(images[i]);
		}


		Stitcher::Status status = stitcher.stitch(inputImages, stitchedImages);
		inputImages.clear();
			
		if (status != Stitcher::OK) {
			ROS_FATAL("Unable to stitch images together!, exiting now");
			break;
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
				not stop the program since we're in a ros loop
				
				Only ctrl+c will stop the loop, but this is terrible because the code
				is forced to quit and it will never be able to release the webcams
				with .release() - This causes the webcams to not connect the next time
				this program is executed, without having to physically re-connected it again
				
			*/
			imshow("Stiching Window", stitchedImages);
			waitKey(25);
			destroyWindow("Stiching Window");
			ROS_INFO("Destroyed stitch image window");
		}
		ROS_INFO("Counter: %d", counter);
	}
	
	//If you Ctrl+C while inside the ros::ok loop, this part will never get executed
	ROS_INFO("Releasing VideoCapture objects!");
	
	for(int i = 0; i < CAMERA_AMOUNT; i++){
		cameras[i].release();
	}
	
	ROS_INFO("All VideoCaptures released, proper shutdown complete");
	
	return 0;
	
}
