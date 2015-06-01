#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const static std::string CV_WINDOW = "Image Subscriber Example";
const static std::string TOPIC = "image";

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
	static unsigned int counter = 1;
	try{
		ROS_INFO("Recieved an image for the %d time", counter);		
		cv::imshow(CV_WINDOW, cv_bridge::toCvShare(msg, "bgr8")->image);
		if(cv::waitKey(30) == 27){
			ROS_INFO("Shutdown event recieved");
			ros::shutdown();
		}
	}catch (cv_bridge::Exception& e){
		ROS_ERROR("Could not convert from '%s' to 'mono8'.", msg->encoding.c_str());
	}
	counter++;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "image_subscriber_example");
	ros::NodeHandle nh;

	cv::namedWindow(CV_WINDOW);

	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe(TOPIC, 1, imageCallback);
	
	ros::spin();
	cv::destroyWindow(CV_WINDOW);
	return 0;
}
