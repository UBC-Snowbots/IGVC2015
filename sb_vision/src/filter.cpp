// filter.cpp
// Purpose: 
// Author: Jannicke Pearkes
#include <iostream>
#include <vector>
#include <stdlib.h>
#include "filter.h"



int const kMaxBinary = 255;
int const kThreshold = 180; //195

using namespace std;

filter::filter():
     kBlur(13)
 {
 	
 }

void filter::testPrint(void)
{
	std::cout << "Filter test print \n" << std::endl;
}

cv::Mat filter::getUpdate(cv::Mat inputImage) 
{
	
	//Blur Image
	if (kBlur % 2 == 0) kBlur = kBlur + 1; //Will crash if kBlur is even
	cv::medianBlur(inputImage, imageBlur, kBlur);

	//Create grey-scaled version of image
	cv::cvtColor(imageBlur, imageGrey, CV_RGB2GRAY);

	cv::medianBlur(imageGrey, imageBlur2, kBlur);

	//Threshold the image: 3 different options
	//Regular threshold
	threshold(imageBlur2, imageThresholded, kThreshold, kMaxBinary,
			cv::THRESH_BINARY);
	//Adaptive threshold
	//adaptiveThreshold(image_blur2, image_thresholded,255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,71, 15);
	//Otsu threshold
	//threshold(image_blur2, image_thresholded, threshold_value, 255 , THRESH_OTSU|THRESH_BINARY  );
	//image_direction = image_thresholded.clone();
	outputImage = imageThresholded.clone();
	//Canny Edge detection
	//cv::Canny(image_thresholded, image_canny, 50, 200, 3);
	
	//outputImage = inputImage.clone();

	return outputImage;
}

void filter::displayImages(cv::Mat inputImage)
{
	    std::cout<<"size of input image"<<inputImage.rows<<","<<inputImage.cols<<endl;
        std::cout<<"size of input outputImage"<<outputImage.rows<<","<<outputImage.cols<<endl;
	    //Display Input Image
	    cv::namedWindow("Display Image", CV_WINDOW_NORMAL);
		cvMoveWindow("Display Image", 0, 300);
		cv::imshow("Display Image", inputImage);
        
	    //Display direction image
		cv::namedWindow("Direction", CV_WINDOW_NORMAL);
		cvMoveWindow("Direction", 400, 0);
		cv::imshow("Direction", outputImage);
        cout<<"displayed the images"<<endl; 
}