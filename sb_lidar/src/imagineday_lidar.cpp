#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sb_msgs/CarCommand.h>

using namespace ros;
using namespace std;

//Global constatns
static const double PI		 = 3.1415265;
static const double IGNORE_ANGLE = PI/4; //45degrees
static const int    OFFSET_RAYS = 30;        // offset from central ray
static const double REDZONE      = 1.0; // only rotate, do not go
static const double ORANGEZONE   = 1.0; // turn 
static const double SLOW_SPEED	 = 0.1;
static const double SPEED_LIMIT  = 0.3;

//ros related constants
static const string NODE_NAME       = "imagine_lidar";

static const string SUBSCRIBE_TOPIC = "scan";
static const string PUBLISH_TOPIC   = "lidar_nav";
static int LOOP_FREQ = 30;

sb_msgs::CarCommand car_command;

int danger = 0;

//call back function
void lidar_callback(const sensor_msgs::LaserScanConstPtr& msg_ptr) {

	int num_rays = msg_ptr->ranges.size();
	car_command.priority = 1;
	float range_min = msg_ptr->range_min;

	for(int i = 0; i < num_rays;i++)
	{
		float angle = msg_ptr->angle_min + i*(msg_ptr->angle_increment);
		float dist = msg_ptr->ranges[i];

		/*if (dist <= REDZONE)
		{
			// stop moving forward
			car_command.throttle = 0;
			if (angle <= 0) { car_command.steering = 0.3; }
			else { car_command.steering = -0.3; }		
		}*/
		if (dist <= ORANGEZONE)
		{
			car_command.throttle = 0.1;
			if (angle <= 0) { car_command.steering = 0.3; }
			else { car_command.steering = -0.3; }	
		}
		else
		{
			car_command.throttle = SPEED_LIMIT;
			car_command.steering = 0;
		}
	}
}


int main (int argc, char** argv)
{
	init(argc, argv,NODE_NAME);
	NodeHandle n;
	

	Subscriber lidar_state = n.subscribe(SUBSCRIBE_TOPIC,20,lidar_callback);
	
	Publisher car_pub = n.advertise<sb_msgs::CarCommand>(PUBLISH_TOPIC,1);
	
	Rate loop_rate(LOOP_FREQ);
	ROS_INFO("ready to go");
	
	ROS_INFO("going");
	while(ros::ok())
	{
		car_pub.publish(car_command);
		ROS_INFO("Throttle: %0.2f, Steering: %0.2f", car_command.throttle, car_command.steering);
	  	ros::spinOnce();
    		loop_rate.sleep();	
	}
  return 0;
}


