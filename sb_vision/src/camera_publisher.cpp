#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <signal.h>

static const std::string ENCODING = "bgr8";
static const std::string TOPIC = "image";

//If only one camera is connected, the ID is always 0
cv::VideoCapture cap(0);


//Handles the exiting of the publisher
void onShutdown(int sig){
	cap.release();
	ROS_INFO("Camera should have been released, proper shutdown complete");
	ros::shutdown();
}


int main(int argc, char** argv){
	ROS_INFO("Starting");
	ros::init(argc, argv, "image_publisher_example", ros::init_options::NoSigintHandler);
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::Publisher pub = it.advertise(TOPIC, 1);

	//Links shutdown process to our custom shutdown function
	signal(SIGINT, onShutdown);

	//Check if video device can be opened with the given index
	if(!cap.isOpened()){
		ROS_FATAL("There was a problem connecting to the camera");
		ROS_FATAL("If this persist, please run usb-reset.sh");
		return -1;
	}
		
	cv::Mat frame;
	sensor_msgs::ImagePtr msg;

	ros::Rate loop_rate(5);

	while (nh.ok()) {
		cap.read(frame);

		if(!frame.empty()) {
			ROS_INFO("Publishing image");
			msg = cv_bridge::CvImage(std_msgs::Header(), ENCODING, frame).toImageMsg();
			pub.publish(msg);
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}
