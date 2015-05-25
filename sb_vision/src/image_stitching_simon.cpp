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
	ROS_INFO("Initializing Webcams");
	//camera.set(CV_CAP_PROP_FPS,15);
	//camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	//camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	for (int portNumber = 3; portNumber >= 0; portNumber--){
		ROS_INFO("Attempting port: %d", portNumber);
		if (camera.open(portNumber)){
			ROS_INFO("Connection established on port: %d", portNumber);
			return true;
		}
	}

	ROS_FATAL("Unable to establish connection on ports");
	
	return false;
}

int main(int argc, char **argv)	
{
	init(argc, argv, NODE_NAME);

	VideoCapture cap1;
	VideoCapture cap2;
	VideoCapture cap3;
									
	if(!connectCamera(cap1) || !connectCamera(cap2) || !connectCamera(cap3)){
		ROS_FATAL("Unable to connect to all cameras, exiting now");
		return 0;
	}
							
	Mat image1, image2, image3;
	Stitcher stitcher = Stitcher::createDefault();
	int counter = 0;
	namedWindow("Stiching Window");

	while (ros::ok() && counter < 10 && getchar() == -1){
		ROS_INFO("Image Stitching Started!");
		
		counter++;
		
		cap1 >> image1;
		cap2 >> image2;
		cap3 >> image3;

		if (!cap1.read(image1)){
			ROS_ERROR("Cannot read image 1 from video stream");
			//continue;
		}				
			 
		if (!cap2.read(image2)){
			ROS_ERROR("Cannot read image 2 from video stream");
			//continue;
		}
			
		if (!cap3.read(image3)){
			ROS_ERROR("Cannot read image 3 from video stream");
			//continue;
		}
		
		Mat pano;
		vector<Mat> imgs;
		/*
			What is .data() doing?, what about using .empty() to
			determine if the matrix is empty or not
		*/	
		if (image1.empty() || image2.empty() || image3.empty()){
			ROS_WARN("One of the Mat is empty");
			//continue; 
		}

		imgs.push_back(image1);
		imgs.push_back(image2);
		imgs.push_back(image3);

		Stitcher::Status status = stitcher.stitch(imgs, pano);
			
		imgs.pop_back();
		imgs.pop_back();
		imgs.pop_back();
			
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
			waitKey(25);
			destroyWindow("Stiching Window");
			ROS_INFO("Destroyed stitech image window");
		}
		ROS_INFO("Counter: %d", counter);
	}
	
	//If you Ctrl+C while inside the ros::ok loop, this part will never get executed
	ROS_INFO("Releasing VideoCapture objects!");
	cap1.release();
	cap2.release();
	cap3.release();
	ROS_INFO("All VideoCapture released, goodbye!");
	
	return 0;
	
}
