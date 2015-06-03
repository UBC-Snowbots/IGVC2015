#include "image_to_grid.hpp"

void chatterCallback(const sensor_msgs::Image::ConstPtr& msg)
{	
	ConvertSensorImage hello(msg->header, msg->height,msg->width,msg->encoding,msg->is_bigendian,msg->step, (msg->data));
	hello.convert();
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imagelistener");
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("test_image", 1000, chatterCallback);
	ros::spin();
	return 0;	
}
