// Image filter class
// Purpose: Applies the filters to the image to extract the white lines
// Author: Jannicke Pearkes

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/video.hpp>
#include <opencv/cv.h>
#include <iostream>


class filter
{ 
    public:
       filter();
       cv::Mat outputImage;
       cv::Mat getUpdate(cv::Mat inputImage); 
       void displayImages(cv::Mat inputImage);
       void testPrint(void);
       //cv::Mat inputImage;

    private:
       int kBlur;
       
       cv::Mat imageBlur;
       cv::Mat imageBlur2;
       cv::Mat imageGrey;
       cv::Mat imageThresholded;
};
