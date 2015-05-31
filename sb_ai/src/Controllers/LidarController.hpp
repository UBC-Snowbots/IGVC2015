#pragma once

#include <unistd.h>
#include <sys/time.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sb_msgs/CarCommand.h>
#include "ControllerBase.hpp"

namespace ai{

class LidarController: public ControllerBase{

private:
	// some flags
	int danger;
	int backup;

	// some variables
	long timeDiffce;	// currently in milliseconds
	struct timeval current_time;
	struct timeval prev_time;

	// data types
	geometry_msgs::Vector3 directions;
	geometry_msgs::Vector3 prevObjectDist;
	geometry_msgs::Vector3 currentObjectDist;
	geometry_msgs::Vector3 velocity;
	sb_msgs::CarCommand car_command;
    
    ros::Subscriber lidar_state;

public:
	LidarController(ros::NodeHandle&);
	double clamp (double in, double cap);
	
	int GetClockSpeed();
	
	geometry_msgs::Twist Update();

	void callback(const sensor_msgs::LaserScanConstPtr& msg_ptr);

    geometry_msgs::Twist twist_converter(sb_msgs::CarCommand cc);

    geometry_msgs::Vector3 convertPolar(double r, double theta);

    geometry_msgs::Vector3 calculateVelocity(geometry_msgs::Vector3 dist1, geometry_msgs::Vector3 dist2);

};


}
