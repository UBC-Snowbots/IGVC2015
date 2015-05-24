#pragma once

#include <unistd.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sb_msgs/CarCommand.h>

namespace ai{

//Global constatns
static const double PI		 = 3.1415265;
static const double IGNORE_ANGLE = PI; 		//ignore rays beyond this angle
static const int    OFFSET_RAYS = 30;        // offset from central ray
static const double REDZONE      = 1;			// originally 0.5
static const double ORANGEZONE   = 1.5;			// originally 1
static const double SLOW_SPEED	 = 0.15;		// originally 0.1
static const double NORMAL_SPEED  = 0.25;
static const double SLOW_TURN	 = 0.15;
static const double SPEED_LIMIT  = 0.3; 		// originally 0.3
static const double TURN_LIMIT  = 0.4;
static const double THROTTLE_CONST = -1;
static const double STEERING_CONST  = -2;
static const unsigned int MICROSECOND = 2000000;	// sleep time

//ros related constants
static const string NODE_NAME       = "lidar_node";
static const string SUBSCRIBE_TOPIC = "scan";
static const string PUBLISH_TOPIC   = "lidar_nav";
static int LOOP_FREQ = 30;

// user defined data types
geometry_msgs::Vector3 directions;
sb_msgs::CarCommand car_command;

class LidarController: public ControllerBase{

private:
	// some flags
	int danger;
	int backup;

	// data types
	geometry_msgs::Vector3 directions;
    sb_msgs::CarCommand car_command;

public:
	double clamp (double in, double cap);

	void callback(const sensor_msgs::LaserScanConstPtr& msg_ptr);

    geometry_msgs::Twist twist_converter(sb_msgs::CarCommand cc);
};


}
