/*
	UBC Snowbots IGVC 2015

	Purpose: Vision Demo for GeeringUP presentation in July, this demo allows the user
	to adjust the trackbar to find the right colors of filering.

	Author: Simon Jinaphant
	Future Reference: http://opencv-srf.blogspot.ca/2010/09/object-detection-using-color-seperation.html
*/

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <signal.h>

const static std::string CONTROLS_WINDOW = "Filter Controls";
const static std::string ORIGINAL_WINDOW = "Oringinal";

using namespace cv;

VideoCapture cap(0);

void onShutdown(int sig){
	destroyWindow(CONTROLS_WINDOW);
	destroyWindow(ORIGINAL_WINDOW);
	cap.release();
	ROS_INFO("All objects should have been released, proper shutdown complete");
}

int main( int argc, char** argv ){
	signal(SIGINT, onShutdown);

	if(!cap.isOpened()){
		ROS_FATAL("UNABLE TO OPEN CONNECTION TO CAMERA");
		return -1;
    }

    namedWindow(CONTROLS_WINDOW, CV_WINDOW_AUTOSIZE); //create a window called CONTROLS_WINDOW
    namedWindow(ORIGINAL_WINDOW, CV_WINDOW_AUTOSIZE); //create a window called CONTROLS_WINDOW

	int iLowH = 170;
	int iHighH = 179;

	int iLowS = 150; 
	int iHighS = 255;

	int iLowV = 60;
	int iHighV = 255;

	//Create trackbars in CONTROLS_WINDOW window
	createTrackbar("LowH", CONTROLS_WINDOW, &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", CONTROLS_WINDOW, &iHighH, 179);

	createTrackbar("LowS", CONTROLS_WINDOW, &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", CONTROLS_WINDOW, &iHighS, 255);

	createTrackbar("LowV", CONTROLS_WINDOW, &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", CONTROLS_WINDOW, &iHighV, 255);

	 
	int errorCounter = 0;
	while (errorCounter < 3){
		Mat imgOriginal;
	       // bool bSuccess = cap.read(imgOriginal); // read a new frame from video
		if (!cap.read(imgOriginal)){
	    	ROS_WARN("Unable to read frame, trying again");
	    	errorCounter++;
	    	continue;
		}

		Mat imgHSV, imgThresholded;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
	  	
	 	//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
	
		imshow(ORIGINAL_WINDOW, imgOriginal); //show the original image
		imshow(CONTROLS_WINDOW, imgThresholded); //show the original image
		
		if(waitKey(30) == 27){
			ROS_INFO("SHUTING DOWN PROGRAM NOW!");
			cap.release();
			break; 
	    }

	}

   return 0;
}
