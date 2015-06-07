#include "image_to_grid.hpp"

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{	
	ConvertSensorImage hello(msg->header, msg->height,msg->width,msg->encoding,msg->is_bigendian,msg->step, (msg->data));
	hello.convert();
	hello.printMap();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imagelistener");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("image_bird", 1000, chatterCallback);
	ros::spin();
	return 0;	
}
