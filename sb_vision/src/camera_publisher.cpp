/*
	UBC Snowbots IGVC 2015

	Purpose: This is an exmaple code for publishing an ROS image when read
	from a connect camera via OpenCV.
	
	This node requires two extra ROS packages: cv_bridge and image_transport
	BY DEFAULT YOU SHOULD HAVE IT ALREADY INSTALLED, DO NOT ATTEMPT TO UPDATE
	THE PACKAGES AS THE NEWER VERSION IS INCOMPATIBLE WITH SOME ROS DEPENDENCIES

	Author: Simon Jinaphant
*/

#include <ros/ros.h>
#include <signal.h>
#include <opencv2/opencv.hpp>
#include <opencv2/stitching/stitcher.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "Camera.h"

static const std::string ENCODING = "bgr8";
static const std::string TOPIC = "image";
static const unsigned int CAMERA_AMOUNT = 3;

std::vector<Camera> cams;
std::vector<cv::Mat> frames;


//Handles the exiting of the publisher
void onShutdown(int sig){
	for(int i = 0; i < CAMERA_AMOUNT; i++){
		cams[i].release();
	}
	ROS_INFO("Camera should have been released, proper shutdown complete");
	ros::shutdown();
}


int main(int argc, char** argv){
	using namespace cv;
	ROS_INFO("Starting the Publisher");
	ros::init(argc, argv, "image_publisher_example", ros::init_options::NoSigintHandler);
	ros::NodeHandle nodeHandler;
	
	//Set up image_transport for publishing/subscribing images via compressed formats
	image_transport::ImageTransport imageTransporter(nodeHandler);
	image_transport::Publisher transportPub = imageTransporter.advertise(TOPIC, 1);

	//Links shutdown process to our custom shutdown function
	signal(SIGINT, onShutdown);


	for(int i = 0; i < CAMERA_AMOUNT; i++){
		cams.push_back(Camera(i));
		frames.push_back(Mat());
	}

	sensor_msgs::ImagePtr message;
	ros::Rate loop_rate(5);

	//Could possibly take out @imgs as its redundant...
	std::vector<Mat> imgs;
	Stitcher stitcher = Stitcher::createDefault(true);

	while (nodeHandler.ok()) {
		for(int i = 0; i < CAMERA_AMOUNT; i++){
			cams[i].awaitFrame(frames[i]);
			imgs.push_back(frames[i]);
		}

		Mat panorama;
		Stitcher::Status status = stitcher.stitch(imgs, panorama);
		imgs.clear();

		if (status != Stitcher::OK) {
			ROS_FATAL("Unable to stitch images together!, trying again...");
			continue;
		}else{
			ROS_INFO("Publishing image");
			message = cv_bridge::CvImage(std_msgs::Header(), ENCODING, panorama).toImageMsg();
			transportPub.publish(message);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
