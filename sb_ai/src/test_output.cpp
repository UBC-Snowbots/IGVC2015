#include "ros/ros.h"
#include <string>
#include <iostream>
#include <geometry_msgs/Twist.h>

using namespace std;

static const string NODE_NAME = "ai_test_output";
static const string CAR_COMMAND_TOPIC = "lidar_nav";

int main(int argc, char **argv)
{
	ros::init(argc, argv, NODE_NAME);
	ros::NodeHandle n;
	ros::Publisher car_pub = n.advertise<geometry_msgs::Twist>(CAR_COMMAND_TOPIC, 100);
	ros::Rate loop_rate(10);  // 10 hz

	int count = 0;
	geometry_msgs::Twist twist;

	while (ros::ok()) {
		
		if (count < 30) {
			twist.linear.x = 0;
			twist.linear.y = 1;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
		}

		else {
			twist.linear.x = 1;
			twist.linear.y = 0;
			twist.linear.z = 0;
			twist.angular.x = 0;
			twist.angular.y = 0;
			twist.angular.z = 0;
		}
		cout << "Lin x: " << twist.linear.x << endl;
		cout << "Lin y: " << twist.linear.y << endl;
		car_pub.publish(twist);
		loop_rate.sleep();
	}
	return 0;
}
