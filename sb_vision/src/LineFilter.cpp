// UBC Snowbots
// LineFilter.cpp
//		Purpose: Takes in an image from the camera and filters
//				the image to isolate white lines on the field
// Author: Jannicke Pearkes


#include "ros/ros.h" 
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Namespaces
using namespace cv;
using namespace ros; 
using namespace std; 

// Global Variables
static const string NODE_NAME = "LineFilter";
static const string PUBLISH_TOPIC = "image";
const int MSG_QUEUE_SIZE = 20;
int windowNumber = 0;

//Functions
void displayImage(Mat, const char* );
Mat loadImage(void);
Mat filterImage(Mat);
Mat showHistogram(const Mat &, int , int );
Mat showHSVHistograms(Mat);

// Input: an image and a name of the image
// Function: displays image in a named window and automatically tiles the image
// Return: Nothing
void displayImage(Mat image, const char* nameOfImage)
{
	//Spacing can vary depending on screen resolution, current set up for Mecanum 
	const int xSpacing = 400;
	const int ySpacing = 300;
	int xPosition = 0;
	int yPosition = 0; 

	if(! image.data ) cout <<  "Could not open or find the image" << endl;
    else
    {
    	//Calculate where to place window
    	int row = floor((windowNumber/4));
    	int column = windowNumber -4*row;
    	xPosition = column*xSpacing;
    	yPosition = row*ySpacing;
    	namedWindow( nameOfImage, WINDOW_NORMAL);	// Create a window for display.
    	cvMoveWindow(nameOfImage, xPosition, yPosition); // Place window in required position 
    	windowNumber++;
    	imshow( nameOfImage, image );   
    }
    return;
}

//Load image from specified file
Mat loadImage(void)
{	
 	Mat image;
    image = imread("/home/mecanum/Pictures/linefollower.jpg", CV_LOAD_IMAGE_COLOR);   // Read the file specified by full path
 	return(image);
}

// Input:Image of field 
// Function: Filters the image to isolate white lines
Mat filterImage(Mat image)
{
	Mat filteredImage;
	int blurValue = 4;
	//Blur Image
	if (blurValue % 2 == 0)
		blurValue = blurValue + 1;
	medianBlur(image, filteredImage, blurValue);

	image = showHSVHistograms(filteredImage);

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
	
	return(filteredImage);
	
}
/*
//Outputs a histogram of values 
cv::Mat showHistogram(const Mat &inImage, int numBins, int numChannels) {

		cv::MatND hist;
		// For a gray scale [0:255] we have 256 bins
		const int bins[1] = { numBins };
		const float hranges[2] = { 0.0, numBins / 1.0 };
		const float* ranges[1] = { hranges };
		const int channels[1] = { numChannels };

		cv::calcHist(&inImage, 1, // histogram from 1 image only
				channels, cv::Mat(), // no mask is used
				hist, // the output histogram
				1, // 1D histogram
				bins, ranges // pixel value ranges
				);

		// Get min and max bin values
		double maxVal = 0;
		double minVal = 0;
		cv::minMaxLoc(hist, &minVal, &maxVal, 0, 0);
		// The image to display the histogram
		cv::Mat histImg(bins[0], bins[0], CV_8U, cv::Scalar(255));

		// Map the highest point to 95% of the histogram height to leave some
		// empty space at the top
		const int histHeight = bins[0];
		const int maxHeight = 0.95 * histHeight;

		cv::Mat_<float>::iterator it = hist.begin<float>();
		cv::Mat_<float>::iterator itend = hist.end<float>();

		int barPosition = 0;
		for (; it != itend; ++it) {
			float histValue = (*it);
			int barHeight = (histValue * maxHeight) / maxVal;
			cv::line(histImg,
					// start the line from the bottom, and go up based on the barHeight
					// Remember the (0,0) is the top left corner
					cv::Point(barPosition, histHeight),
					cv::Point(barPosition, histHeight - barHeight),
					cv::Scalar::all(0));
			barPosition++;
		}
		return histImg;
	}*/

	Mat showHSVHistograms(Mat image) {

		vector<Mat> channels;
		Mat image_histo = image.clone();
		//cvtColor(image, image_histo, COLOR_RGB2HSV);
		cvtColor(image, image_histo, COLOR_RGB2HSV);
		split(image_histo, channels);

		// And then if you like
		Mat image_H = channels[0];
		Mat image_S = channels[1];
		Mat image_V = channels[2];
		displayImage(image_H,"Hue Image \0");
		displayImage(image_S,"Saturation Image \0");
		displayImage(image_V,"Value Image \0");


/*
		// Calculate histograms of image
		//histogram_H = showHistogram2(image_H);
		histogram_H = showHistogram4(image_H, 180, 0);
		threshold(image_H, image_HThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

		//histogram_S = showHistogram3(image_S);
		histogram_S = showHistogram4(image_S, 255, 0);
		threshold(image_S, image_SThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

		histogram_V = showHistogram4(image_V, 255, 0);
		threshold(image_V, image_VThresh, 0, 255, THRESH_OTSU | THRESH_BINARY);

		Mat inGreenRange;
		Mat testRGB;
		//void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
		//cout<<image_H<<endl;

		inRange(image_H, lowerBound,upperBound,image_thresholded);
*/

		//Split image into RGB
		vector<Mat> RGBchannels;
		Mat image_BnoG;
		//cvtColor(image, image_HSV,CV_RGB2HSV);
		//inRange(image_H,50, 70, testRGB);
		split(image, RGBchannels);
		Mat image_R = RGBchannels[0];
		Mat image_G = RGBchannels[1];
		Mat image_B = RGBchannels[2];
		Mat imageTest;

		displayImage(image_R,"Red Image \0");
		displayImage(image_B,"Blue Image \0");
		displayImage(image_G,"Green Image \0");


		subtract(image_G,image_H, imageTest);
		displayImage(imageTest,"Image G subtract Hue Image \0");


		int lowerBound = 60;
		int upperBound = 170;
		inRange(image_H, lowerBound,upperBound,imageTest);
		displayImage(imageTest,"Image B subtract G Image \0");

		//subtract(image_R,image_B, imageTest);
		//displayImage(imageTest,"Image R subtract Blue Image \0");

		//subtract(image_R, image_G, image_noG);
		//subtract(image_R, image_CV_HOUGH_STANDARDG, image_noG);
		//subtract(image_R, image_G, image_noG);
		//subtract(image_B, image_G, image_BnoG);
		//add(image_noG,image_BnoG, image_noG);
		//threshold(image_noG, image_thresholded, 7, max_BINARY_value, THRESH_BINARY);

		//merge(RGBchannels, image_noG);
		return imageTest;

	}


// Looking to implement an iterated fitting algorithm here
Mat fit(Mat image)
{
	/*//Ehhh this is not nice at all...
	Point points[image.rows][image.cols];
	Mat image_32;
	std::vector<Point2f> vec;
	//Mat image_H = channels[0];
	inRange(image, 180,255,image_32);
	//image.convertTo(image_32,CV_32F);
	displayImage(image_32,"right before loop");
	
	image_32=image_32+1;
	cout<<"hello"<<endl;
	for (int i = 0; i < image_32.rows; i++) {
		for (int j = 0; j < image_32.cols; j++) {
			if ((image_32.at<uchar>(i, j)) == 1) {	
				vec.push_back(Point2f((float)i,(float)j));
			}
		}
	}
	Vec4f line;
	cout<<vec;
	Mat src2;
	Mat dst;
	//solve(image_32, src2, dst, DECOMP_LU);
	//cout<<"info: "<<image_32.points.depth()<<endl;
	//fitLine(vec,line,CV_DIST_L2,0,0.01,0.01);
*/
	//points = createPoints(image);
	//fitLine(points);
	//drawLine();
	/*vector<Mat> channels;

	Mat image_histo = image;
	split(image_histo, channels);

		// And then if you like
	Mat image_H = channels[0];
	//vector<Vec2f> lines;
	Mat binary = Mat::zeros(1,0,CV_32F);
	
	inRange(image_H, 180,255,binary);
	displayImage(image,"beforebinary");
	displayImage(binary,"binary");
	//binary.convertTo(binary,CV_32F);
	cv::convertScaleAbs(binary,binary);
    cv::normalize(binary,binary,0,255,cv::NORM_MINMAX);
	//binary=cvtColor(image,CV_32F);
	//std::vector<float> input[4] = [0,2,3,4]; 
	std::vector<cv::Point2f> points;
	/*points.push_back(cv::Point2f(3,3));
	points.push_back(cv::Point2f(4,4));
	points.push_back(cv::Point2f(5,5));
*/
	/*
   vector<vector<int> > ptvec;
for (int i = 0; i < binary.rows; i++)
{
    vector<int> row;    
    //image.row(i).copyTo(row);
    //ptvec.push_back(row);
}

Mat iP = Mat::zeros(1,0,CV_32F);

int sz = iP.cols*iP.rows;
vector<Point2f> ptvec = Mat_ <Point2f>(iP);
vector<Point2f> ptvec = Mat_ <Point2f>(iP.reshape(1, sz));

	Vec4f line;
	cv::fitLine(ptvec,line,CV_DIST_L2,0,0.01,0.01);
	//cv::fitLine(points,line,CV_DIST_L2,0,0.01,0.01);

	//fitLine(image, lines, CV_DIST_L2, 0, 0.001, 0.001);
	//cout<<lines[2];
	// std::cout << "The contents of fifth are:";
    //for (std::vector<Vec4f>::iterator it = line.begin(); it != line.end(); ++it)
    //  std::cout << ' ' << *it;
    //  std::cout << '\n';
	//Point P1=Point(lines[2],lines[3]);
	//Point P2=Point(lines[0],lines[1]);
	//line( image, P2, P1,Scalar( 0, 0, 0 ),2,8 );
		//Point pt1, pt2;
		//pt1.x = 0;
		//pt1.y = 0;
		//pt2.x = 100;
		//pt2.y = 100;
		//line(image, pt1, pt2, CV_RGB(0, 0, 0), 4, CV_AA);
		//line(image_out, pt1, pt2, CV_RGB(250, 100, 255), 1, CV_AA);
		*/
		return image;
	}

int main(int argc, char **argv)
{

	init(argc, argv, NODE_NAME); //Initialize ROS node
	NodeHandle n;
	//Publisher filter_pub = n.advertise<sensor_msgs::ImagePtr>(PUBLISH_TOPIC, MSG_QUEUE_SIZE);
	sensor_msgs::ImagePtr msg;
	Mat image = loadImage(); //Load sample image from file
	int goF = 0; //goodness of fit
	displayImage(image, "image1\0"); //Display raw image
	image = filterImage(image); // Run a filter to isolate white lines from grass
	image = fit(image);
 	displayImage(image, "filtered image\0"); //Display raw image
 	while (ros::ok())
    {
   	// Wait for one ms, break if escape is pressed
    	//msg = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image);
    	//filter_pub.publish(msg);
   	if (waitKey(100) == 27) return(0); 
    }

return 0;
}