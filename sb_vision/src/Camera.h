#pragma once

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>
#include "ros/ros.h"
#include <iostream>

class Camera{
private:
	cv::VideoCapture camera;
	unsigned int frameReadError; 
	static const unsigned int CAMERA_FPS = 15;
	static const unsigned int CAMERA_WIDTH = 960;
	static const unsigned int CAMERA_HEIGHT = 540;
	

public:
	Camera(unsigned int);
	virtual ~Camera();
	void release();

	bool getFrame(cv::Mat&);
	void awaitFrame(cv::Mat&);
};
