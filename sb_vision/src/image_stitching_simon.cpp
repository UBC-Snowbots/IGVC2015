#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream> 
#include <vector>

using namespace ros;
using namespace cv; 
using namespace std;

static const string NODE_NAME = "Image stitching - Simon";
static const unsigned int CAMERA_AMOUNT = 3;

/*
This function intialize the connection to each webcam
	
The deviceID for all three webcams will be between [1,3]
Once a connection is established, the connected camera will occupied 
the lowest deviceID avalible, making the next connection ID predictable
*/
bool connectCamera(VideoCapture& camera){
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

	ROS_FATAL("Unable to establish connection to webcam %d", occupiedID);
	return false;
}


int main(int argc, char **argv)	{
	init(argc, argv, NODE_NAME);

	/*
	When I tried putting the VideoCapture and Mat objects inside
	an array/vector, the program kept losing connection frequently.
	Once connection is lost the device cannot is release() regardless
	and a hard restart will be required to re-execute the program again

	An internal error message will be printed out whenever this happens, 
	but there are no ways to catch the error as far as I'm aware
	*/

	VideoCapture cap1;
	VideoCapture cap2;
	VideoCapture cap3;
									
	if(!connectCamera(cap1) || !connectCamera(cap2) || !connectCamera(cap3)){
		ROS_FATAL("Unable to connect to all cameras, exiting now");
		ROS_FATAL("If this error persist, please disconnect all webcams or restart the computer");
		return 0;
	}
							
	Mat image1, image2, image3;
	Stitcher stitcher = Stitcher::createDefault(true);

	int counter = 0;
	namedWindow("Stiching Window");

	while (ros::ok() && counter < 5){
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
		cap1 >> image1;
		cap2 >> image2;
		cap3 >> image3;

		if (!cap1.read(image1))
			ROS_ERROR("Cannot read image 1 from video stream");			 
		if (!cap2.read(image2))
			ROS_ERROR("Cannot read image 2 from video stream");	
		if (!cap3.read(image3))
			ROS_ERROR("Cannot read image 3 from video stream");
		
		Mat pano;
		vector<Mat> imgs;

		if (image1.empty() || image2.empty() || image3.empty())
			ROS_WARN("One of the Mat is empty");

		imgs.push_back(image1);
		imgs.push_back(image2);
		imgs.push_back(image3);

		Stitcher::Status status= stitcher.stitch(imgs, pano);
		imgs.clear();
			
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
			not stop the program since we're in a loop
				
			Only ctrl+c will stop the loop, but this is terrible because the code
			is forced to quit and it will never be able to release the webcams
			with .release() - This causes the webcam to not connect the next time
			this program is executed, without having to manually re-connected it again
			*/
			imshow("Stiching Window", pano);
			waitKey(50);
			destroyWindow("Stiching Window");
			ROS_INFO("Destroyed stitch image window");
		}
		ROS_INFO("Counter: %d", counter);
	}
	
	//If you CTRL + C while inside the ros::ok loop, this part will never get executed
	//causing a re-connection problem to the webcams in any subsequent execution.
	
	ROS_INFO("Releasing VideoCapture objects!");
	cap1.release();
	cap2.release();
	cap3.release();
	ROS_INFO("All VideoCaptures released, proper shutdown complete");
	
	return 0;
}