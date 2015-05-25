#pragma once

#include <unistd.h>
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

	// data types
	geometry_msgs::Vector3 directions;
    sb_msgs::CarCommand car_command;
    
    ros::Subscriber lidar_state;

public:
	LidarController(ros::NodeHandle&);
	double clamp (double in, double cap);
	
	int GetClockSpeed();
	
	geometry_msgs::Twist Update();

	void callback(const sensor_msgs::LaserScanConstPtr& msg_ptr);

    geometry_msgs::Twist twist_converter(sb_msgs::CarCommand cc);
};


}
