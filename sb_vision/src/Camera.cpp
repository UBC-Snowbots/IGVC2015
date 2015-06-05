#include "Camera.h"
#include <stdexcept>
Camera::Camera(unsigned int deviceID)
{
	camera.set(CV_CAP_PROP_FPS, CAMERA_FPS);
	frameReadError = 0;
	/*
	Our webcams support 1080p recording, making the ideal resolution 1920x1080
	However this large resolution will significantly impact the sticher's 
	performance, therefore we need to determine an ideal resolution
	*/
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
	if(!camera.open(deviceID)){
		ROS_FATAL("Unable to establish connection to device ID: %d", deviceID);
		ROS_FATAL("If there's an error above saying:");
		ROS_FATAL("\tV4L index N is not correct");
		ROS_FATAL("change the deviceID, otherwise run usb-reset.sh");
		//camera.release();
		//ros::shutdown();
		throw std::invalid_argument("Failed to init camera");
	}
}


Camera::~Camera(){
	release();
}

void Camera::release(){
	camera.release();
}


bool Camera::getFrame(cv::Mat& frame){
	if (!camera.read(frame)){
		ROS_WARN("Unable to get retrieve a frame");
		return false;
	}
	if (frame.empty()){
		ROS_WARN("Frame is empty");
		return false;
	}
	return true;
}


void Camera::awaitFrame(cv::Mat& frame){
	while(!getFrame(frame) && frameReadError < 10){
		//ROS_INFO("Awaiting...");
		frameReadError++;
	}
	if(frameReadError == 10){
		ROS_FATAL("Too much errors reading from this iteration");
	}
	frameReadError = 0;
}
