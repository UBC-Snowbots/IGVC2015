/*
 * UBC Snowbots
 * Pylon_Detector.cpp
 *        
 * Program for detecting and covering up striped color objects
 * Part of code taken and modified from the webpage below
 * http://stackoverflow.com/questions/16492471/multiple-color-object-detection-using-opencv
 * Code for reading a video file
 * Taken and modified from:
 * http://opencv-srf.blogspot.ca/2011/09/capturing-images-videos.html
 */

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#define TRUE 1
#define FALSE 0
#define SHOW_POINTS 0

using namespace ros;
using namespace cv;
using namespace std;

// Function prototypes
void analyseStream( Mat );
Mat filterColour( Mat );
vector<Rect> retrieveContours( Mat );
void determineIfRectRelated( Mat, Rect, Rect );
Point findMidpoint( Rect rectA );
Rect drawBoundingRect( Mat, Rect, Rect );

static const string NODE_NAME = "Pylon_Detector";
const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv){
    //Stuff to run once 
    init(argc, argv, NODE_NAME); //initializes your ROS node
     while (ros::ok()){
    	VideoCapture cap;

	// If no input arguments, capture from primary camera	
	if( argc == 1 ){
		cout << "Input from primary camera" << endl;
		VideoCapture camera(0);
		cap = camera;
	}
	// If incorrect number of arguments, exit the program
	else if( argc != 2 ){
		cout << "ERROR. Incorrect number of arguments." << endl;
		cout << "Options for running the program are listed below:" << endl;
		cout << "For computer's primary camera:" << endl;
		cout << "\t./AnalyseImageVideo" << endl;
		cout << "For analysing an image" << endl;
		cout << "\t./AnalyseImageVideo <PathofImageFile>" << endl;
		cout << "For analysing a video file" << endl;
		cout << "\t./AnalyseImageVideo <PathofVideoFile>" << endl;
		cout << "Program terminating..." << endl;
		exit(0);
	}
	// Input is an image
	else if( !imread(argv[1]).empty() ){
	    	cout << "Input file is a picture" << endl;
		Mat inputImage = imread(argv[1]);
		analyseStream( inputImage );

		waitKey(0);
		exit(0);
	}
	// Input is a video
	else{
		// If video file specified, capture from that video
		VideoCapture inputVideo( argv[1] );
		cap = inputVideo;
	}

	// If not successfull opening file, exit program
   	if ( !cap.isOpened() ){ 
       	 	cout << "Cannot open the video file" << endl;
	 	return -1;
    	}

	// Get the frames per seconds of the video
    	double fps = cap.get(CV_CAP_PROP_FPS); 
     	cout << "Frame per seconds : " << fps << endl;

/*	// Create a window called "MyVideo"
    	namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); 
*/
   	while(1){
		Mat frame;

		// Read a new frame from video
		bool bSuccess = cap.read(frame); 

		// If reading frame not successful, break loop
		if (!bSuccess){
		     	cout << "Cannot read the frame from video file" << endl;
		     	break;
		}
		analyseStream( frame );

	/*	// Show the frame in "MyVideo" window
		imshow("MyVideo", frame); 
*/
		// wait for 'esc' key press for 30 ms. 
		// If 'esc' key is pressed, then break loop
		if(waitKey(30) == 27){
			cout << "esc key is pressed by user" << endl; 
			break; 
	       	}
    	}
    }
    return 0;
}

/* 
 * Processes an image or an individual frame from a video,
 * filters the image for the colour of the pylon
 * and draws bounding boxes on the pylon
 * Param: input image or video frame
 */
void analyseStream( Mat inputImage ){
	// Display input image
	cout << "Reading image successful\n";
	namedWindow("Input Image", WINDOW_AUTOSIZE );
    	imshow("Input Image", inputImage);

	// Filter color of input image
	Mat filteredImage( inputImage.size(), CV_8UC3 );	
	filteredImage = filterColour( inputImage );

	// Determine rectangles that bound the filtered color
	vector<Rect> boundRect = retrieveContours( filteredImage );

	Mat canvas = inputImage.clone();

	// Determine if rectangles are related
	for( int i = 0; i < boundRect.size() ; i++ ){
		Rect currRect = boundRect[i];
		for( int i = 0; i < boundRect.size(); i++ ){
			Rect nextRect =  boundRect[i];
			determineIfRectRelated( canvas, currRect, nextRect );
		}
	}
	
	// Display image with the bounding boxes image
	cout << "Processing image successful\n";
	namedWindow("Bounding Box", WINDOW_AUTOSIZE );
	imshow("Bounding Box", canvas);

	return;
}

/*
 * Filters out the color of interest (red)
 * Param: input image
 * Return: filtered image
 */
Mat filterColour( Mat inputImage ){
	Mat filteredImage;
	inRange(inputImage, Scalar(40, 0, 180), Scalar(135, 110, 255), filteredImage);
	
	// namedWindow("Filtered Image", WINDOW_AUTOSIZE );
	// imshow("Filtered Image", filteredImage);

	return filteredImage;
}

/*
 * Retrieve the contours of the filtered image
 * Draw boxes to outline the contours
 * Param: filtered image
 * Return: vector of rectanges that bound the color of interest
 */
vector<Rect> retrieveContours( Mat filteredImage ){
	vector< vector<Point> > contours;
	vector< Vec4i > hierarchy;
	findContours( filteredImage, 
		          contours, 
		          hierarchy, 
		          CV_RETR_TREE, 
		          CV_CHAIN_APPROX_SIMPLE, 
		          Point(0, 0) );

	vector< vector<Point> > contours_poly( contours.size() );
	vector<Rect> boundRect( contours.size() );
	for( int i = 0; i < contours.size(); i++ )
	    { 
		approxPolyDP( Mat(contours[i]), contours_poly[i], 3, true );
		boundRect[i] = boundingRect( Mat(contours_poly[i]) );
	    }   

	// Debug purposes: draw bonding rects
	Mat tmp = Mat::zeros( filteredImage.size(), CV_8UC3 );
	for( int i = 0; i< contours.size(); i++ )
	  	//rectangle( tmp, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), 2, 8, 0 );
		rectangle( tmp, boundRect[i].tl(), boundRect[i].br(), Scalar(0, 255, 0), CV_FILLED, 8, 0 );
	
	// namedWindow("Bounded Contours", WINDOW_AUTOSIZE );
	// imshow("Bounded Contours", tmp);

	return boundRect;
}

/* 
 * Determines if the 2 input rectangles are related (one closely stacked on top of each other)
 * If they are related, a bounding box is 
 * Param: input image
 * Param: rectange 1
 * Param: rectange 2
 */
void determineIfRectRelated( Mat canvas, Rect rect1, Rect rect2 ){
	// Determine and draw midpoints
	Point rect1Midpoint = findMidpoint( rect1 );
	cout << "Rectangle 1 Midpoint: " << rect1Midpoint << "\n";
	if( SHOW_POINTS ){
		circle(canvas, rect1Midpoint, 5, Scalar(255,255,255), CV_FILLED, 8, 0);
	}

	Point rect2Midpoint = findMidpoint( rect2 );
	cout << "Rectangle 2 Midpoint: " << rect2Midpoint << "\n";
	if( SHOW_POINTS ){	
		circle(canvas, rect2Midpoint, 5, Scalar(255,255,255), CV_FILLED, 8, 0);
	}

	// Determine if the rectangles are related
	// Find vertical distance between these midpoints
	int vertDistMidpts = abs( rect1Midpoint.y - rect2Midpoint.y );
	cout << "Vertical distance between midpoints: " << vertDistMidpts << "\n";
	
	// Calculate the sum of the rectangles' heights	X 2
	int heightsThres = (rect1.height + rect2.height)*2;
	cout << "Sum of rectangle heights: " << heightsThres << "\n";
	
	// Find vertical distance between these midpoints
	int horizDistMidpts = abs( rect1Midpoint.x - rect2Midpoint.x );
	cout << "Horizantal distance between midpoints: " << vertDistMidpts << "\n";
	
	// Calculate the sum of the rectangles' width	
	int widthsThres = rect1.width + rect2.width;
	cout << "Sum of rectangle widths: " << widthsThres << "\n";

	// If this vertical distance between midpoints is less than the sum of the rectangles' heights,	
	// Draw big rectange that encloses both rectangles, 
	// (need to determine left most, right most, top most, bottom most coordinate of the rectangles)
	if( (vertDistMidpts < heightsThres) && (horizDistMidpts < widthsThres) ){
		Rect boundingBox = drawBoundingRect( canvas, rect1, rect2 );

		// Draw filled bounding box
		 rectangle( canvas, boundingBox.tl(), boundingBox.br(), Scalar(0, 0, 0), CV_FILLED, 8, 0 );
		
		/*
		// Draw Translucent bounding box
	    	Mat roi = canvas( boundingBox );
	    	Mat color(roi.size(), CV_8UC3, cv::Scalar(0, 125, 125)); 
	    	double alpha = 0.3;
	    	addWeighted(color, alpha, roi, 1.0 - alpha , 0.0, roi);
		*/ 
	}
	else{
		cout << "Rectangles are unrelated\n";
	}
	
	return;
}

/*
 * Determine midpoint of rectangle
 * Param: rectangle
 * Return: midpoint of the rectangle
 */
Point findMidpoint( Rect rectA ){
	int midXDist, midYDist;
	int midX, midY;
	midXDist = abs( (rectA.tl().x - rectA.br().x)/2 );
	midYDist = abs( (rectA.tl().y - rectA.br().y)/2 );
	
	midX = rectA.tl().x + midXDist;
	midY = rectA.tl().y + midYDist;

	return Point( midX, midY );
}

/* 
 * Construct a bounding rectangle that fully encloses the 2 input rectangles
 * Param: input image
 * Param: rectange 1
 * Param: rectange 2
 * Param: bounding rectange
 */
Rect drawBoundingRect( Mat canvas, Rect rect1, Rect rect2 ){
	int leftMostCoord, rightMostCoord;
	int topMostCoord, bottomMostCoord;

	// Determine left most coordinate
	if( rect1.tl().x < rect2.tl().x ){
		leftMostCoord = rect1.tl().x;
	}
	else{
		leftMostCoord = rect2.tl().x;
	}

	// Determine right most coordinate
	if( rect1.br().x < rect2.br().x ){
		rightMostCoord = rect2.br().x;
	}
	else{
		rightMostCoord = rect1.br().x;
	}

	// Determine top most coordinate
	if( rect1.tl().y < rect2.tl().y ){
		topMostCoord = rect1.tl().y;
	}
	else{
		topMostCoord = rect2.tl().y;
	}
	
	// Determine bottom most coordinate
	if( rect1.br().y < rect2.br().y ){
		bottomMostCoord = rect2.br().y;
	}
	else{
		bottomMostCoord = rect1.br().y;
	}
	
	Point topLeftVertex = Point( leftMostCoord, topMostCoord );
	if( SHOW_POINTS ){
		circle(canvas, topLeftVertex, 5, Scalar(255,255,255), CV_FILLED, 8, 0);
	}
	cout << "Top left coordinate: " << topLeftVertex << "\n";

	Point bottomRightVertex = Point( rightMostCoord, bottomMostCoord );
	if( SHOW_POINTS ){
		circle(canvas, bottomRightVertex, 5, Scalar(255,255,255), CV_FILLED, 8, 0);
	}
	cout << "Bottom right coordinate: " << bottomRightVertex << "\n";
	
	Rect boundingRect = Rect(topLeftVertex, bottomRightVertex);
	
	return boundingRect;
}
