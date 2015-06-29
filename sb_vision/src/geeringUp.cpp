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

const static std::string CV_WINDOW = "Filter Controls";

using namespace cv;
	VideoCapture cap(0);

void onShutdown(int sig){
	destroyWindow(CV_WINDOW);
	cap.release();
	ROS_INFO("All objects should have been released, proper shutdown complete");
}

int main( int argc, char** argv ){
	signal(SIGINT, onShutdown);


	if(!cap.isOpened()){
		ROS_FATAL("UNABLE TO OPEN CONNECTION TO CAMERA");
		return -1;
    }

    namedWindow(CV_WINDOW, CV_WINDOW_AUTOSIZE); //create a window called CV_WINDOW

	int iLowH = 170;
	int iHighH = 179;

	int iLowS = 150; 
	int iHighS = 255;

	int iLowV = 60;
	int iHighV = 255;

	//Create trackbars in CV_WINDOW window
	createTrackbar("LowH", CV_WINDOW, &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", CV_WINDOW, &iHighH, 179);

	createTrackbar("LowS", CV_WINDOW, &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", CV_WINDOW, &iHighS, 255);

	createTrackbar("LowV", CV_WINDOW, &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", CV_WINDOW, &iHighV, 255);

	int iLastX = -1; 
	int iLastY = -1;

	//Capture a temporary image from the camera
	Mat imgTmp;
	cap.read(imgTmp); 

	//Create a black image with the size as the camera output
	Mat imgLines = Mat::zeros( imgTmp.size(), CV_8UC3 );;
	 
	int errorCounter = 0;
	while (errorCounter < 3){
		Mat imgOriginal;
	       // bool bSuccess = cap.read(imgOriginal); // read a new frame from video
		if (!cap.read(imgOriginal)){
	    	ROS_WARN("Unable to read frame, trying again");
	    	errorCounter++;
	    	continue;
		}

		Mat imgHSV;

		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
		Mat imgThresholded;
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image  
	  	
	  	//morphological opening (removes small objects from the foreground)
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

		//morphological closing (removes small holes from the foreground)
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		//Calculate the moments of the thresholded image
		Moments oMoments = moments(imgThresholded);

		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;

		// if the area <= 10000, I consider that the there are no object in the image and it's because of the noise, the area is not zero 
		if (dArea > 10000){
		 	//calculate the position of the ball
		 	int posX = dM10 / dArea;
		 	int posY = dM01 / dArea;        
		        
			if (iLastX >= 0 && iLastY >= 0 && posX >= 0 && posY >= 0){
				//Draw a red line from the previous point to the current Point
				line(imgLines, Point(posX, posY), Point(iLastX, iLastY), Scalar(0,0,255), 2);
			}
			
			iLastX = posX;
			iLastY = posY;
		}

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		
		imgOriginal = imgOriginal + imgLines;
		//imshow("Original", imgOriginal); //show the original image
		if(waitKey(30) == 27){
			ROS_INFO("SHUTING DOWN PROGRAM NOW!");
			cap.release();
			break; 
	    }

	}

   return 0;
}
