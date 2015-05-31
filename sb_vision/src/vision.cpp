// UBC Snowbots
// vision.cpp
//		Purpose: vision node to combine image stitcher, filtering, and bird's eye view transformation
//				
// Author: Jannicke Pearkes


#include "ros/ros.h" 

#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>
#include <opencv2/stitching/stitcher.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <signal.h>

#include "filter.h"
#include "IPM.h"

// Namespaces
using namespace cv;
using namespace ros; 
using namespace std; 

static const string VISION_NODE_NAME = "vision";
static const std::string CVWINDOW = "Sticher Window";
//static const string PUBLISH_TOPIC = "image";
const int MSG_QUEUE_SIZE = 20;

VideoCapture cap1;
VideoCapture cap2;
VideoCapture cap3;
static const unsigned int CAMERA_AMOUNT = 1;

void onShutdown(int sig);
bool connectToCamera(VideoCapture& camera);
int sum_cam(bool camera_status[]);

int main(int argc, char **argv)
{
	ros::init(argc,argv, VISION_NODE_NAME);
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
    image_transport::ImageTransport it(n);
    image_transport::Publisher pub = it.advertise("image", 1);
    sensor_msgs::ImagePtr msg;
	 
    //signal(SIGINT, onShutdown);  



    Mat image1, image2, image3;         
    std::vector<Mat> imgs;
    Stitcher stitcher = Stitcher::createDefault(true);
    int counter = 0;
    int errorCounter = 0;
    namedWindow(CVWINDOW);

    Mat inputImg, inputImgGray;
	Mat outputImg;
    Mat frame;
    Mat pano;
    Mat image_pano;
    bool camera_status[3]; //false means off, true means on
        
    //IPM Set-up, will put into class itself soon
    //int width = 1920;
    //int height = 1080;
    int width = 1920;
    int height = 1080;
    int wf = width/1920; //width scale factor
    int hf = height/1080; //height scale factor
    vector<Point2f> origPoints;
    origPoints.push_back( Point2f(0, height) );
    origPoints.push_back( Point2f(width, height) );
    //origPoints.push_back( Point2f(width/2+450, 580) );
    //origPoints.push_back( Point2f(width/2-250, 580) );
    origPoints.push_back( Point2f(width/2+450*wf, 620*hf) );
    origPoints.push_back( Point2f(width/2-250*wf, 620*hf) );

    // The 4-points correspondences in the destination image
    vector<Point2f> dstPoints;
    dstPoints.push_back( Point2f(0, height) );
    dstPoints.push_back( Point2f(width, height) );
    dstPoints.push_back( Point2f(width, 0) );
    dstPoints.push_back( Point2f(0, 0) );
	
    
    //create camera status array
    camera_status[0] = connectToCamera(cap1);
    camera_status[1] = connectToCamera(cap2);
    camera_status[2] = connectToCamera(cap3);

    if(!connectToCamera(cap1) && !connectToCamera(cap2) && !connectToCamera(cap3))
    ROS_INFO("Unable to connect to any camera, using file as input");
    
    
	while(ros::ok())
    {
        //TODO: add camera capture allowance for any amount of cameras
        // Read in image from video camera, or other source
        
        if(camera_status[0]) cap1 >> image1;
        if(camera_status[1]) cap2 >> image2;
        if(camera_status[2]) cap3 >> image3;
        
        if (!cap1.read(image1)) ROS_WARN("Cannot read image 1 from video stream");
        if (!cap2.read(image2)) ROS_WARN("Cannot read image 2 from video stream");
        if (!cap3.read(image3))ROS_WARN("Cannot read image 3 from video stream");
        
        
        if (image1.empty() || image2.empty() || image3.empty()) {
         ROS_WARN("One of the Mats is empty");   
        }
      
	    

        // Static image for debugging purposes, seem to need an absolute
        // path to the image to avoid crashes
    	//cv::Mat image = imread("/home/jannicke/Pictures/Image1.jpg", 1);
        //if(!image.data) std::cout<<"NO IMAGE DATA"<<std::endl;
        //else std::cout<<"image data exists"<<std::endl;
        //std::cout<<"image read"<<std::endl;

        //Image Stitching
        // If 2 arrays have values
        if(sum_cam(camera_status)>=2)
        {
            ROS_INFO("Image Stitching Started!");
            counter++;
            
            if(camera_status[0])imgs.push_back(image1);
            if(camera_status[1])imgs.push_back(image2);
            if(camera_status[2])imgs.push_back(image3);

            Stitcher::Status status= stitcher.stitch(imgs, pano);
            imgs.clear();
            
            if (status != Stitcher::OK) {
                ROS_FATAL("Unable to stitch images together!, trying again...");
                continue;
            }
            else {                
                ROS_INFO("Awaiting for stiched image to display");
                imshow(CVWINDOW, pano);
                if(waitKey(50) == 27){
                    ROS_INFO("ESC key pressed! Exiting loop now");
                    ROS_WARN("The next run has a higher chance of crashing for unknown reasons");
                    //break;
                    }
            }
        }
        else if(camera_status[0]) pano = image1;
        else if(camera_status[1]) pano = image2;
        else if(camera_status[2]) pano = image3;
        else pano = imread("/home/jannicke/Pictures/Image1.jpg", 1);

        //Apply filter to stitched image
        filter myfilter; //create filter
        image_pano = myfilter.getUpdate(pano);
        myfilter.displayImages(pano);
        
     
	    // Transform image
	    IPM ipm( Size(width, height), Size(width, height), origPoints, dstPoints );
		frame = image_pano;
		if( frame.empty() )
        {
            cout<<"frame empty" <<endl;
        } 
		// Color Conversion
		if(frame.channels() == 3)cvtColor(frame, inputImgGray, CV_BGR2GRAY);				 		 
		else frame.copyTo(inputImgGray);			 		 
		// Process
		ipm.applyHomography( frame, outputImg);		 
		ipm.drawPoints(origPoints, frame );
        
        Size size(80,60);//the dst image size,e.g.100x100
        Mat dst;
        resize(outputImg,dst,size);//resize image

        // Publish Image
        if(!outputImg.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", dst).toImageMsg();
            pub.publish(msg);
            ROS_INFO("Vision published an image");
            cv::waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();

    }

}
    



/*
This function handles the releasing of objects when this node is
requested or forced (via CTRL+C) to shutdown.
*/
void onShutdown(int sig){
    destroyWindow(CVWINDOW);
    cap1.release();
    cap2.release();
    cap3.release();
    ROS_INFO("All objects should have been released, proper shutdown complete");
    ros::shutdown();
}

/*
This function intialize the connection to each webcam
    
The deviceID for all three webcams will be between [1,3]
Once a connection is established, the connected camera will occupied 
the lowest deviceID avalible, making the next connection ID predictable
*/
bool connectToCamera(VideoCapture& camera){
    static unsigned int occupiedID = 1;
    
    camera.set(CV_CAP_PROP_FPS, 20);
    
    //Our webcams support 1080p recording, making the ideal resolution 1920x1080
    //However this large resolution will significantly impact the sticher's performance
    //Therefore we need to determine an ideal resolution
    camera.set(CV_CAP_PROP_FRAME_WIDTH, 960);
    camera.set(CV_CAP_PROP_FRAME_HEIGHT, 540);

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

int sum_cam(bool camera_status[])
{
    int count = 0;
    for (int i=0;i<3;i++)
    {
        if(camera_status[i]) count++;
    }
    return count;
}