#include "ros/ros.h"
#include <iostream>
#include "sb_msgs/AISimGPS.h"

using namespace std;

static const string NODE_NAME = "test_gps";
static const string GPS_PUB_TOPIC = "ai_gps";

sb_msgs::AISimGPS sim_gps;

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	ros::Publisher gps_pub = n.advertise<sb_msgs::AISimGPS>(GPS_PUB_TOPIC, 100);
	ros::Rate loop_rate(10);
	
	sim_gps.latitude = 8;
	sim_gps.longitude = 7;

	while (ros::ok()) {
		cout << "Running.." << endl;
		gps_pub.publish(sim_gps);
		loop_rate.sleep();
	}
}
