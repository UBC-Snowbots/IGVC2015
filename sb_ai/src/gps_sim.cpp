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
	ros::Publisher gps_pub = n.advertise<sb_msgs::AISimGPS>(GPS_PUB_TOPIC, 2);
	ros::Rate loop_rate(2);
	
	sim_gps.latitude = 0;
	sim_gps.longitude = 0;
	sim_gps.theta = 0;

  int count = 0;
  
	while (ros::ok()) {
		
		ROS_INFO("Lat: %ld", sim_gps.latitude); 
		ROS_INFO("Long: %ld", sim_gps.longitude);
		ROS_INFO("Theta: %ld", sim_gps.theta);
		
		gps_pub.publish(sim_gps);
		
		loop_rate.sleep();
		
		if (count < 10)
		{
		  sim_gps.latitude++;
		}
		else if (count < 20)
		{
		  sim_gps.theta++;
		} 
		else
		{
		  sim_gps.latitude++;
		  sim_gps.longitude++;
		}

		count++;
	}
}
