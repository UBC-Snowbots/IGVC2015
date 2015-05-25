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
static unsigned int occupied_camera_id=0;
static const string NODE_NAME = "descriptive_name";
const int MSG_QUEUE_SIZE = 20;

//THE FOLLOWING CODE WAS MODIFIED AND STILL UNTESTED AT RUNTIME

bool connectToCamera(VideoCapture& camera){
	//TODO: Is there a way to tell which port the webcams auto-connect to?

	ROS_INFO("Initializing Webcam #%d", occupied_camera_id);
	camera.set(CV_CAP_PROP_FPS, 30);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    
    //Is it possible for the device to have greater id?!?
	for (int deviceID = occupied_camera_id; deviceID < 5; deviceID++){
		ROS_INFO("Attempting connection to device: %d", deviceID);
		if (camera.open(deviceID)){
			occupied_camera_id++;
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
	std::vector<VideoCapture> cameras(CAMERA_AMOUNT);
	std::vector<Mat> images(CAMERA_AMOUNT);
	Mat stitchedImages;
	//std::vector<Mat> inputImages(3);
	//inputImages.reserve(CAMERA_AMOUNT);

	for(int i = 0; i < CAMERA_AMOUNT; i++){
        if(!connectToCamera(cameras[i])){
            ROS_FATAL("Unable to connect to webcam %d", i);
            ROS_FATAL("If this connection problem persist:");
						ROS_FATAL("Disconnect all devices from the computer and restart :(");
            return -1;
        } else {
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
			ROS_INFO("Reading from camera %d", i);	
			cameras.at(i) >> images.at(i);
			ROS_INFO("Read from camera %d", i);	
			if(!cameras.at(i).read(images.at(i))){
				ROS_ERROR("Cannot capture image on camera %d", i);
			}
				ROS_INFO("Checking read from camera %d", i);	
			if(images.at(i).empty()){
				ROS_ERROR("Capture image on camera %d was empty", i);
			}

			//inputImages.push_back(images.at(i));
			ROS_INFO("finished from camera %d", i);	
		}


		Stitcher::Status status = stitcher.stitch(images, stitchedImages);
		//inputImages.clear();
		images.pop_back();
		images.pop_back();
		images.pop_back();
			
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
			waitKey(50);
			destroyWindow("Stiching Window");
			ROS_INFO("Destroyed stitch image window");
		}
		ROS_INFO("Counter: %d", counter);
	}
	
	//If you Ctrl+C while inside the ros::ok loop, this part will never get executed
	ROS_INFO("Releasing VideoCapture objects!");
	
	for(int i = 0; i < CAMERA_AMOUNT; i++){
		cameras.at(i).release();
	}
	
	ROS_INFO("All VideoCaptures released, proper shutdown complete");
	
	return 0;
	
}
