#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv){
	ROS_INFO("Starting");
	ros::init(argc, argv, "image_publisher");
	ros::NodeHandle nh;
	ROS_INFO(">>>");
	image_transport::ImageTransport it(nh);
	ROS_INFO("^^^");
	image_transport::Publisher pub = it.advertise("stitcher", 1);
	ROS_INFO("...");
	cv::VideoCapture cap(1);
	//Check if video device can be opened with the given index
	if(!cap.isOpened()){
		ROS_FATAL("Cannot connect");
		return 1;
	}
		
	cv::Mat frame;
	sensor_msgs::ImagePtr msg;
	ROS_INFO("Entering Loop");
	ros::Rate loop_rate(5);
	while (nh.ok()) {
		ROS_INFO("Looping");
		cap >> frame;
		cap.read(frame);
		//Check if grabbed frame is actually full with some content
		if(!frame.empty()) {
			ROS_INFO("Sending image...");
			msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
			pub.publish(msg);
			cv::waitKey(1);
		}

	ros::spinOnce();
	loop_rate.sleep();
	}
}
