//-------------------------------------------------

// UBC Snowbots

// image_stitching.cpp

// Purpose: To stitch two images taken from two webcams on a robot

// Author: Rishabh Singal

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <iostream> 
#include <vector>

using namespace ros;
using namespace cv; 
using namespace std;

/*
//----VISION--------
//Initialize camera
	VideoCapture cap; // open the default camera
	//VideoCapture cap("sample-course.avi");
	//VideoCapture cap("roadsample.mov");
	//VideoCapture cap("fieldsample.mov");
	
	for (int i=5; i>=-1; i--)
	{
		cout << "trying port: " << i << endl;
		cap.open(i);
		if (cap.isOpened())
		{
			cout << "YAY!" << endl;
			break;
		}

		if (i ==-1)
		{
			cout << "failed to connect to camera" << endl;
			return -1;
		}
	}
//---END VISION

  while (ros::ok())
  {


//----VISION-----
  
   
   //Read image
    cap >> image; //from video
 
	if (!cap.read(image)) //if not success, break loop
	 {
	   cout << "Cannot read a frame from video stream" << endl;
	  break;
	  }
   }
*/



static const string NODE_NAME = "descriptive_name";

const int MSG_QUEUE_SIZE = 20;

int main(int argc, char **argv)
	
{

 init(argc, argv, NODE_NAME); //initializes your ROS node
 
	while (ros::ok())

		{

			ROS_INFO("Hi!"); // Publishes to log and terminal

			//----VISION--------
			//Initialize cameran
			
			//TODO: Google VideoCapture API, find a close() function or something that closes the connection
			
				VideoCapture cap1; // open the default camera
				VideoCapture cap2;
				VideoCapture cap3;
				//VideoCapture cap("sample-course.avi");
				//VideoCapture cap("roadsample.mov");
				//VideoCapture cap("fieldsample.mov");
	
								
				for (int i=5; i>=-1; i--)
				{
					cout << "trying port: " << i << endl;
					cap1.open(i);
					if (cap1.isOpened())
					{
						cout << "YAY!" << endl;
						cout << i << endl; 
						break;
					}

					if (i ==-1)
					{
						cout << "failed to connect to camera 1" << endl;
						return -1;
					}
				}
				
				for (int j=5; j>=-1; j--)
				{
					cout << "trying port: " << j << endl;
					cap2.open(j);
					if (cap2.isOpened())
					{
						cout << "YAY!" << endl;
						cout << j << endl; 
						break;
					}

					if (j ==-1)
					{
						cout << "failed to connect to camera 2" << endl;
						return -1;
					}
				}
				
				
				for (int k=10; k>=-1; k--)
				{
					cout << "trying port: " << k << endl;
					cap3.open(k);
					if (cap3.isOpened())
					{
						cout << "YAY!" << endl;
						cout << k << endl; 
						break;
					}

					if (k == -1)
					{
						cout << "failed to connect to camera 3" << endl;
						return -1;
					}
				}
				
				
							
				 //Read image
				Mat image1;
				cap1 >> image1; //from video
				
				Mat image2;				    
				cap2 >> image2; //from video
				
				Mat image3;				    
				cap3 >> image3; //from video
			 
				if (!cap1.read(image1)) //if not success, break loop
				 {
					  cout << "Cannot read image 1 (before while loop) frame from video stream" << endl;
					  break;
				 }				
						 
				if (!cap2.read(image2)) //if not success, break loop
				  {
					  cout << "Cannot read image 2 frame (before while loop) from video stream" << endl;
					  break;
				  }
				
				if (!cap3.read(image3)) //if not success, break loop
				  {
					  cout << "Cannot read image 3 frame (before while loop) from video stream" << endl;
					  break;
				  }
				  
				   
	
			//---END VISION
			
			// Put all your other code in here
			//  Mat fr1 = imread("/home/rish/snowbots_ws/src/IGVC2015/sb_vision/src/a.jpg",0);
			//  Mat fr1 = imread(image); 
			Mat pano;
			vector <Mat> imgs;
			       
			 
			/*if (!fr1.data || !fr2.data){

					cout << "NO OPEN" <<std::endl ;
					return -1; 
			}
			*/
			
			if (!image1.data || !image2.data || !image3.data){

				cout << "NO OPEN (before while loop)" <<std::endl ;
				return -1; 
			}

			Stitcher stitcher = Stitcher::createDefault(); // The value you entered here is the default

			while(true){
									
				//Mat image1;
				cap1 >> image1; //from video    
				cap2 >> image2; //from video			    
				cap3 >> image3; //from video
				
				/*if (!image1.data || !image2.data || !image3.data){
					cout << "NO OPEN" << std::endl ;
					return -1; 
				}
				*/
				
				//Mat image3;				    
				//cap3 >> image3; //from video
			 
				if (!cap1.read(image1)){
					  cout << "Cannot read a image 1 frame from video stream" << endl;
					  break;
				 } else if (!cap2.read(image2)){
					  cout << "Cannot read image 2 frame from video stream" << endl;
					  break;
				  }/*else if (!cap3.read(image3)){
					  cout << "Cannot read image 3 frame from video stream" << endl;
					  break;
				  }
				  */

				//cap1 >> image1; //from video
				//cap2 >> image2; //from video
				imgs.push_back(image1);
				imgs.push_back(image2);
				imgs.push_back(image3);
				
				Stitcher test = Stitcher::createDefault(/*try_use_gpu*/);
				//Stitcher::Status status = test.stitch(imgs, pano);
				Stitcher::Status status = stitcher.stitch(imgs, pano);
				
				/*imshow("Video 1", image1);
				imshow("Video 2", image2);
				imshow("Video 3", image3);*/
				
				imgs.pop_back();
				imgs.pop_back();
				imgs.pop_back();
				

					if (status != Stitcher::OK) {
						cout << "Error stitching - Code: " << int(status)<< endl;
						//return -1;
					}else{			    
						//imshow("Video 2", image2);
						imshow("Stitched Video", pano);
					}
	
					waitKey(5);
			}
			return 0;

		} //for the first while loop
	return 0;
	
}

//int startVideoCapture (int numPorts, int )




