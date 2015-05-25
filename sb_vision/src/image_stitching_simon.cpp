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
static const unsigned int CAMERA_AMOUNT = 3;
const int MSG_QUEUE_SIZE = 20;

/*
	Assigns a VideoCapture object to an active webcam.
	The deviceID for all three webcams will be between [1,3]
	Once a connection is made, the lowest ID will be occupied
*/
bool connectCamera(VideoCapture& camera){
	//Keeps track of the lowest deviceID to save time
	static unsigned int occupiedID = 1;
	
	camera.set(CV_CAP_PROP_FPS, 20);
	camera.set(CV_CAP_PROP_FRAME_WIDTH, 960);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, 540);

	//deviceIDs shouldn't go pass 3...there's not enough unit testing to confirm this
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

int main(int argc, char **argv)	
{
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
		ROS_FATAL("If this error persist, please disconnect all webcams or restart computer");
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
			There is still a possiblity of an error being produced
			in this section of code. However it seems cv::Exception cannot
			catch them as they aren't OpenCV errors.

			At random times the following error message can show:
				libv4l2: error queuing buf 0: No such device
				libv4l2: error queuing buf 1: No such device
				libv4l2: error dequeuing buf: No such device
				VIDIOC_DQBUF: No such device

			In the next iteration will cause the following error messages
				libv4l2: error dequeuing buf: No such device
				VIDIOC_DQBUF: No such device
				Segmentation fault(core dumped)
		*/
	
		//According to the doc, >> does the same as .read() BUT
		//without doing >> first, the first run of this code will always fail
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

		if (image1.empty() || image2.empty() || image3.empty()){
			ROS_WARN("One of the Mat is empty");
			//continue; 
		}

		imgs.push_back(image1);
		imgs.push_back(image2);
		imgs.push_back(image3);

		Stitcher::Status status;
		try{
			status = stitcher.stitch(imgs, pano);
		} catch (cv::Exception& e){
			//Can't seem to catch any exception
			ROS_FATAL("OpenCV exception detected while stitching, exiting now");
			break;
		}	
		 
		//TODO: Test if clear() will have any problems...
		imgs.clear();
			
		if (status != Stitcher::OK) {
			ROS_FATAL("Unable to stitch images together!, exiting now");
			ROS_FATAL("If this is your first time running, try again!");
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
	
	//If you Ctrl+C while inside the ros::ok loop, this part will never get executed
	ROS_INFO("Releasing VideoCapture objects!");
	cap1.release();
	cap2.release();
	cap3.release();
	ROS_INFO("All VideoCaptures released, proper shutdown complete");
	
	return 0;
	
}
