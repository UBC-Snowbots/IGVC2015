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
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

static const std::string ENCODING = "bgr8";
static const std::string TOPIC = "image";

//If only one camera is connected, the ID is ALWAYS 0
cv::VideoCapture camera(0);


//Handles the exiting of the publisher
void onShutdown(int sig){
	camera.release();
	ROS_INFO("Camera should have been released, proper shutdown complete");
	ros::shutdown();
}


int main(int argc, char** argv){
	ROS_INFO("Starting the Publisher");
	ros::init(argc, argv, "image_publisher_example", ros::init_options::NoSigintHandler);
	ros::NodeHandle nodeHandler;

	//Set up image_transport for publishing/subscribing images via compressed formats
	image_transport::ImageTransport imageTransporter(nodeHandler);
	image_transport::Publisher transportPub = imageTransporter.advertise(TOPIC, 1);

	//Links shutdown process to our custom shutdown function
	signal(SIGINT, onShutdown);

	//Check if video device can be opened with the given index
	if(!camera.isOpened()){
		ROS_FATAL("There was a problem connecting to the camera");
		ROS_FATAL("If this persist, please run usb-reset.sh");
		return -1;
	}
		
	cv::Mat frame;
	sensor_msgs::ImagePtr message;

	ros::Rate loop_rate(5);

	while (nodeHandler.ok()) {
		camera.read(frame);

		if(!frame.empty()) {
			ROS_INFO("Publishing image");
			message = cv_bridge::CvImage(std_msgs::Header(), ENCODING, frame).toImageMsg();
			transportPub .publish(message);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
