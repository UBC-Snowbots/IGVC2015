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
       cv::vector<cv::Mat> channels;
<<<<<<< HEAD
=======
       cv::vector<cv::Mat> channels_RGB;
>>>>>>> b42b066edfed19ed8991346da278d2dc4cfbc435
       cv::Mat imageBlur;
       cv::Mat imageBlur2;
       cv::Mat imageGrey;
       cv::Mat image_histo;
       cv::Mat image_H;
       cv::Mat image_S;
<<<<<<< HEAD
       cv::Mat image_V;     
=======
       cv::Mat image_V; 
       cv::Mat image_R;
       cv::Mat image_G;
       cv::Mat image_B;      
>>>>>>> b42b066edfed19ed8991346da278d2dc4cfbc435
       cv::Mat imageThresholded;
       cv::Mat imageErode;
       cv::Mat imageDilate;
       cv::Mat imageBoth;
       cv::Mat imageBoth2;
       cv::Mat element;
};
