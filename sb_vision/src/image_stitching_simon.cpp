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
static unsigned int occupiedID = 0;
static const string NODE_NAME = "descriptive_name";
const int MSG_QUEUE_SIZE = 20;

//THE FOLLOWING CODE WAS MODIFIED AND STILL UNTESTED AT RUNTIME
bool readFromCamera(VideoCapture& camera, Mat& image, std::vector<Mat>& storage){
	ROS_INFO("Reading from camera...");	
	camera >> image;
	ROS_INFO("Read from camera");	
	if(!camera.read(image)){
		ROS_ERROR("Cannot capture image on camera");
	}
	ROS_INFO("Checking read from camera");
	if(image.empty()){
		ROS_ERROR("Capture image on camera empty");
	}

	storage.push_back(image);
	ROS_INFO("finished from camera");	
	
}

bool connectToCamera(VideoCapture& camera){
	camera.set(CV_CAP_PROP_FPS, 30);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	for (int deviceID = occupiedID; deviceID < 5; deviceID++){
		ROS_INFO("Attempting connection to device: %d", deviceID);
		if (camera.open(deviceID)){
			occupiedID++;
			return true;
		}
	}
	ROS_FATAL("Unable to connect to Webcam #%d", occupiedID);
	return false;
}

int main(int argc, char **argv)	
{
	init(argc, argv, NODE_NAME);
    
    std::cout << "OpenCV version: " << CV_VERSION_MAJOR << "-" << CV_VERSION_MINOR << std::endl; 

	Stitcher stitcher = Stitcher::createDefault();
	VideoCapture camera1, camera2, camera3;
	Mat image1, image2, image3;
	Mat stitchedImages;
	
	std::vector<Mat> inputImages(3);

	connectToCamera(camera1);
	connectToCamera(camera2);
	connectToCamera(camera3);
	
	int counter = 0;
	namedWindow("Stiching Window");

	ROS_INFO("Entering ROS loop...");
	while (ros::ok() && counter < 5){
		ROS_INFO("Image Stitching Started!");	
		counter++;
		

		readFromCamera(camera1, image1, inputImages);
		readFromCamera(camera2, image2, inputImages);
		readFromCamera(camera3, image3, inputImages);

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
			waitKey(50);
			destroyWindow("Stiching Window");
			ROS_INFO("Destroyed stitch image window");
		}
		ROS_INFO("Counter: %d", counter);
	}
	
	//If you Ctrl+C while inside the ros::ok loop, this part will never get executed
	ROS_INFO("Releasing VideoCapture objects!");
	camera1.release();	
	camera2.release();
	camera3.release();
	ROS_INFO("All VideoCaptures released, proper shutdown complete");
	
	return 0;
	
}
