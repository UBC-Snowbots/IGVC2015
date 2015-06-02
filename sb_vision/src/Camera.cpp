#include "Camera.h"

Camera::Camera(unsigned int deviceID=0)
{
	camera.set(CV_CAP_PROP_FPS, CAMERA_FPS);
	
	/*
	Our webcams support 1080p recording, making the ideal resolution 1920x1080
	However this large resolution will significantly impact the sticher's 
	performance, therefore we need to determine an ideal resolution
	*/
	camera.set(CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH);
	camera.set(CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT);
	if(!camera.open(deviceID)){
		ROS_FATAL("Unable to establish connection to device ID: %d", deviceID);
	}
}


Camera::~Camera(){
	camera.release();
	ROS_INFO("All VideoCapture should be closed now");
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
	while(!getFrame(frame)){
		ROS_INFO("Awaiting...");
	}
}
