// UBC Snowbots
// LineFilter.cpp
//		Purpose: Takes in an image from the camera and filters
//				the image to isolate white lines on the field
// Author: Jannicke Pearkes


#include "ros/ros.h" 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <iostream>
#include <string>

// Namespaces
using namespace cv;
using namespace ros; 
using namespace std; 

// Global Variables
static const string NODE_NAME = "LineFilter";
const int MSG_QUEUE_SIZE = 20;
int windowNumber = 0;


// Input: an image and a name of the image
// Function: displays image in a named window and automatically tiles the images
void displayImage(Mat image, const char* nameOfImage)
{
	const int xSpacing = 400;
	const int ySpacing = 300;
	int xPosition = 0;
	int yPosition = 0; 

	if(! image.data ) cout <<  "Could not open or find the image" << endl;
    else
    {
    	namedWindow( nameOfImage, WINDOW_NORMAL);	// Create a window for display.
    	xPosition = (windowNumber)*xSpacing;
    	//yPosition = (windowNumber%4)*ySpacing;
    	yPosition = 0;
    	cvMoveWindow(nameOfImage, xPosition, yPosition);
    	cout<< xPosition<<endl;
    	cout<<windowNumber<<endl;
    	windowNumber++;
    	imshow( nameOfImage, image );   
    }
    return;
}

//Load image from specified file
Mat loadImage(void)
{
 	
 	Mat image;
    image = imread("/home/jannicke/Pictures/fieldsample.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file specified by full path
 	return(image);
}

Mat filterImage(Mat image)
{
	/*
	Mat filteredImage;
	blurValue = 4;
	//Blur Image
	if (blurValue % 2 == 0)
		blurValue = blurValue + 1;
	
	//medianBlur(image, image_blur, blur_value);

	//showHSVHistograms(image_blur);

	//Create grey-scaled version of image
	//cvtColor(image_blur, image_grey, CV_RGB2GRAY);

	//Blur the image
	//if (blur_value % 2 == 0)
	//	blur_value = blur_value + 1;
	//medianBlur(image_grey, image_blur2, blur_value);

	//Threshold the image: 3 different options
	//Regular threshold
	//threshold(image_blur2, image_thresholded, threshold_value, max_BINARY_value,
	//		THRESH_BINARY);
	//Adaptive threshold
	//adaptiveThreshold(image_blur2, image_thresholded,255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,71, 15);
	//Otsu threshold
	//threshold(image_blur2, image_thresholded, threshold_value, 255 , THRESH_OTSU|THRESH_BINARY  );
	//image_direction = image_thresholded.clone();
	//Canny Edge detection
	//Canny(image_thresholded, image_canny, 50, 200, 3);
	
	return(fliteredImage);
	*/
}


int main(int argc, char **argv)
{
	init(argc, argv, NODE_NAME); //Initialize ROS node
	
	Mat image = loadImage(); //Load sample image from file
	displayImage(image, "image1\0"); //Display raw image
	displayImage(image, "image2\0"); //Display raw image
	displayImage(image, "image3\0");
	displayImage(image, "image4\0");
	//filterImage(image); // Run a filter to isolate white lines from grass
 	
 	while (ros::ok())
    {
   	// Wait for one ms, break if escape is pressed
   	if (waitKey(100) == 27) return(0); 
    }

return 0;
}