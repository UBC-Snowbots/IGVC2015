#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

using namespace ros;
using namespace std;

static const double SLOW_SPEED	 = 0.1;
static const double SPEED_LIMIT  = 0.3;

//ros related constants
static const string NODE_NAME       = "driver_test";
static const string PUBLISH_TOPIC   = "lidar_nav";
static int LOOP_FREQ = 10;

geometry_msgs::Twist car_command;

int main (int argc, char** argv)
{
	init(argc, argv,NODE_NAME);
	NodeHandle n;
	
	Publisher car_pub = n.advertise<geometry_msgs::Twist>(PUBLISH_TOPIC,1);
	
	Rate loop_rate(LOOP_FREQ);
	ROS_INFO("ready to go");
	
	// initialize test variables
	int count = 0;
	car_command.linear.y = 1;
	car_command.angular.z = 0;

	ROS_INFO("going");
	while(ros::ok())
	{
		if (count > 20) { car_command.linear.y = 0.5; car_command.angular.z = 0.5; }
		if (count > 40) { car_command.linear.y = 0.0; car_command.angular.z = 0.0; }
		if (count > 60) { car_command.linear.y = -0.5; car_command.angular.z = -0.5; }
		if (count > 80) { car_command.linear.y = -1.0; count = 0; car_command.angular.z = -1.0; }
		car_pub.publish(car_command);
		ROS_INFO("Throttle: %0.2f, Steering: %0.2f", car_command.linear.y, car_command.angular.z);
	  ros::spinOnce();
    loop_rate.sleep();
		++count;	
	}
  return 0;
}


