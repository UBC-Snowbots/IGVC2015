#include <vector>
#include <stdlib.h>
#include <cv.h>
#include <highgui.h>

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>
#include <iostream>

static const int kMaxBinary = 255;
static const int kThreshold = 195;
static const int kBlur = 9; 	//Will crash if kBlur is even

cv::Mat imageBlur;
cv::Mat imageBlur2;
cv::Mat imageGrey;
cv::Mat imageThresholded;

using namespace std;
using namespace cv;

cv::Mat getUpdate(cv::Mat& inputImage){
	
	//Blur Image
	cv::medianBlur(inputImage, imageBlur, kBlur);

	//Create grey-scaled version of image
	cv::cvtColor(imageBlur, imageGrey, CV_RGB2GRAY);

	cv::medianBlur(imageGrey, imageBlur2, kBlur);

	//Threshold the image: 3 different options
	//Regular threshold
	//threshold(imageBlur2, imageThresholded, kThreshold, kMaxBinary, cv::THRESH_BINARY);
	
	//Adaptive threshold
	//adaptiveThreshold(imageBlur2, image_thresholded,255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY,71, 15);
	
	//Otsu threshold
	//threshold(image_blur2, image_thresholded, threshold_value, 255 , THRESH_OTSU|THRESH_BINARY  );

	//image_direction = image_thresholded.clone();
	///outputImage = imageThresholded.clone();
	//Canny Edge detection
	//cv::Canny(image_thresholded, image_canny, 50, 200, 3);
	
	//outputImage = inputImage.clone();

	//return outputImag;
}

void display(cv::Mat& inputImage){
    std::cout<<"size of input image"<<inputImage.rows<<","<<inputImage.cols<<endl;
    //Display Input Image
    cv::namedWindow("Display Image", CV_WINDOW_NORMAL);
	cvMoveWindow("Display Image", 0, 0);
	cv::imshow("Display Image", inputImage);
	cv::waitKey(0);

}

int main(){
	Mat img = imread("/home/simon/Documents/Snowbots2015/src/IGVC2015/sb_vision/src/Image1.jpg", 1);
	getUpdate(img);
	display(imageBlur2);
	return 0;
}
