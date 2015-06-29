// filter.cpp
// Purpose: 
// Author: Jannicke Pearkes
#include <iostream>
#include <vector>
#include <stdlib.h>
#include "filter.h"



int const kMaxBinary = 255;
int const kThreshold = 195; //195
int const kUpperBound = 255;
int const kLowerBound = 200;
int const erosion_elem = 2;
int const erosion_size = 1;
int const dilation_elem = 0;
int const dilation_size = 10;
int const max_elem = 2;
int const max_kernel_size = 101;

using namespace std;

filter::filter():
     kBlur(11)
 {
 	
 }

void filter::testPrint(void)
{
	std::cout << "Filter test print \n" << std::endl;
}

cv::Mat filter::getUpdate(cv::Mat inputImage, int mapping) 
{
	image_histo = inputImage.clone();
	cv::cvtColor(inputImage, image_histo, CV_RGB2HSV);
	
	//Blur Image
	if (kBlur % 2 == 0) kBlur = kBlur + 1; //Will crash if kBlur is even
	cv::medianBlur(image_histo, imageBlur, kBlur);
    cv::medianBlur(inputImage, imageBlur2, kBlur);

	//cvtColor(image, image_histo, COLOR_RGB2HSV);

	split(imageBlur, channels);

    // And then if you like
	image_H = channels[0];
	image_S = channels[1];
	image_V = channels[2];

    cout<<"filter split image into HSV"<<endl;

	split(imageBlur2, channels_RGB);

    // And then if you like
	image_R = channels_RGB[0];
	image_G = channels_RGB[1];
	image_B = channels_RGB[2];

	
    cout<<"filter split image into RGB"<<endl;
    
    inRange(image_H, 0,10,image_H);
    inRange(image_S, 170,255,image_S);
    inRange(image_V, kLowerBound,kUpperBound,image_V);

    inRange(image_R, kLowerBound,kUpperBound,image_R);
    //inRange(image_G, kLowerBound,kUpperBound,image_G);
    //inRange(image_G, 170,255,image_G); //cloudy weather
    inRange(image_G, 200,255,image_G); //sample
    //inRange(image_G,0,200,image_G);
    //inRange(image_G, 230,255,image_G); //sunny weather
    inRange(image_B, 230,255,image_B);

	//cv::medianBlur(imageGrey, imageBlur2, kBlur);
	//Extract green
	//cv::bitwise_and(image_B,image_G,image_V );((
	//if(mapping)
	//{
	//cv::bitwise_not(image_S,image_S);
	//cv::bitwise_and(image_S,image_G,image_G);
    
    //}
    //inRange(image_V, kLowerBound,kUpperBound,imageThresholded);

    // Dilate
    int erosion_type;
    if( erosion_elem == 0 ){ erosion_type = cv::MORPH_RECT; }
    else if( erosion_elem == 1 ){ erosion_type = cv::MORPH_CROSS; }
    else if( erosion_elem == 2) { erosion_type = cv::MORPH_ELLIPSE; }

    element = getStructuringElement( erosion_type,
                                       cv::Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       cv::Point( erosion_size, erosion_size ) );

    /// Apply the erosion operation
    erode( image_B, imageErode, element );

	  int dilation_type;
	  if( dilation_elem == 0 ){ dilation_type = cv::MORPH_RECT; }
	  else if( dilation_elem == 1 ){ dilation_type = cv::MORPH_CROSS; }
	  else if( dilation_elem == 2) { dilation_type = cv::MORPH_ELLIPSE; }

	  element = getStructuringElement( dilation_type,
	                                       cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
	                                       cv::Point( dilation_size, dilation_size ) );
	  /// Apply the dilation operation
	  dilate( image_B, imageDilate, element );

	  erode(imageDilate,imageBoth, element);
	  dilate(imageErode,imageBoth2, element);

    // Erode
    // White and not green
    //Mat bin_hue = (chan[2]<20)|(chan[2]>100);  // note: single '|'
    //Mat bin_sat = (chan[1]<72)|(chan[1]>155);
    //Mat bin_val = (chan[0]<64)|(chan[1]>155);

    // White and not green
    //Mat bin_red = (chan[2]<20)|(chan[2]>100);  // note: single '|'
    //Mat bin_grn = (chan[1]<72)|(chan[1]>155);
    //Mat bin_blu = (chan[0]<64)|(chan[1]>155);

    //Mat bin = bin_red & bin_grn & bin_blu;  // where all conditions were met
	//Threshold the image: 3 different options
	//Regular threshold
	//threshold(imageBlur2, imageThresholded, kThreshold, kMaxBinary,
	//		cv::THRESH_BINARY);
	//Adaptive threshold
	//adaptiveThreshold(image_blur2, image_thresholded,255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,71, 15);
	//Otsu threshold
	//threshold(image_blur2, image_thresholded, threshold_value, 255 , THRESH_OTSU|THRESH_BINARY  );
	//image_direction = image_thresholded.clone();
	outputImage = image_G.clone(); // orginal image G testing fake driver only
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
		cvMoveWindow("Display Image", 0, 0);
		cv::imshow("Display Image", inputImage);
     
	    //Display direction image
		cv::namedWindow("Direction", CV_WINDOW_NORMAL);
		cvMoveWindow("Direction", 0, 300);
		cv::imshow("Direction", outputImage);
   /*
        
        //Display direction image
		cv::namedWindow("Image H", CV_WINDOW_NORMAL);
		cvMoveWindow("Image H", 400, 0);
		cv::imshow("Image H", image_H);
   

        //Display direction image
		cv::namedWindow("Image S", CV_WINDOW_NORMAL);
		cvMoveWindow("Image S", 400, 300);
		cv::imshow("Image S", image_S);
    

        //Display direction image
		cv::namedWindow("Image V", CV_WINDOW_NORMAL);
		cvMoveWindow("Image V", 400, 600);
		cv::imshow("Image V", image_V);

		  //Display direction image
		cv::namedWindow("Image R", CV_WINDOW_NORMAL);
		cvMoveWindow("Image R", 800, 0);
		cv::imshow("Image R", image_R);
   */

        //Display direction image
		cv::namedWindow("Image G", CV_WINDOW_NORMAL);
		cvMoveWindow("Image G", 800, 300);
		cv::imshow("Image G", image_G);
    
/*
        //Display direction image
		cv::namedWindow("Image B", CV_WINDOW_NORMAL);
		cvMoveWindow("Image B", 800, 600);
		cv::imshow("Image B", image_B);

		cv::namedWindow("Image Erode", CV_WINDOW_NORMAL);
		cvMoveWindow("Image Erode", 1200, 0);
		cv::imshow("Image Erode", imageErode);
 
 		cv::namedWindow("Image Dilate", CV_WINDOW_NORMAL);
		cvMoveWindow("Image Dilate", 1200, 300);
		cv::imshow("Image Dilate", imageDilate);

		cv::namedWindow("Image Erode Dilate", CV_WINDOW_NORMAL);
		cvMoveWindow("Image Erode Dilate", 1600, 0);
		cv::imshow("Image Erode Dilate", imageBoth2);

		cv::namedWindow("Image Dilate Erode", CV_WINDOW_NORMAL);
		cvMoveWindow("Image Dilate Erode", 1600, 300);
		cv::imshow("Image Dilate Erode", imageBoth);
		*/
		
}