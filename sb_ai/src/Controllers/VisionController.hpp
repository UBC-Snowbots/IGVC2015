#pragma once

#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sb_msgs/CarCommand.h"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <string>
#include <sstream>
#include <stdio.h>
#include <iostream>
#include <ostream>
#include <stdlib.h>
#include <math.h>
#include <vector>
#include <fstream>
#include <limits>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/video/video.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include "ControllerBase.hpp"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>


namespace ai{

class VisionController: public ControllerBase
{
  private:
    cv::Mat image, image_grey, image_filter, image_thresholded, image_canny, image_blur,
	    image_blur2, image_direction;
    cv::Mat image_H, image_S, image_V, image_R, image_G, image_B, image_histo, image_HThresh, image_SThresh,
	    image_VThresh, image_noG;
    cv::Mat histogram_H, histogram_S, histogram_V;
    cv::Mat image_direction32;

    int blur_value;
    int dx;
    int dy;
    float steeringOut;
    float steering;
    float steeringIncrement;
    float lowsteeringIncrement;
    int priority;
    float direction;
    int noLinesWait;
    float throttle;
    float lowThrottle;
    int count;
    int anyLines;
    geometry_msgs::Twist twist;
    image_transport::Subscriber sub;

    //image_transport::ImageTransport imageTransporter(ros::NodeHandle& nh);
    //image_transport::Subscriber transportSub;

  public:
    VisionController(ros::NodeHandle &nh);
    geometry_msgs::Twist Update();

    void imageCallback( const sensor_msgs::Image::ConstPtr& );
    int getVisionStatus(void);
    void detectLines(void);
    void findcentre(int);
    int findcentre1(int);
    int findcentre2(int);
    void drawLine(int);
    int countLines(int, bool);
    void dump(const cv::Mat&, const char*);
    cv::Mat showHistogram(const cv::Mat&);
    cv::Mat showHistogram2(const cv::Mat&);
    cv::Mat showHistogram3(const cv::Mat&);
    cv::Mat showHistogram4(const cv::Mat &inImage, int, int);
    void showHSVHistograms(cv::Mat);
    void displayWindows(void);
    void filterImage(void);
    void getDirection(void);
    float calculateDirection1();
    float chiSquared(cv::Mat,cv::Mat, float, float, int, int);
    void simpleDir();
  };

}

