#pragma once

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace ai{

class ControllerBase{
public:
	/**
	 *	Override this to do your set-up (registering handlers and such)
	**/
	ControllerBase(){}
	/**
	 *	Override this if you want to specify a specific rate at which you want update() called.
	**/
	virtual int get_clock_speed() { return 10; }
	/**
	 *	This method is called by the main loop approximately once every get_clock_speed() ms.
	 *	Anything you want to run continuuously goes here (you should return your Twist messages so they can be published)
	**/
	virtual geometry_msgs::Twist update() = 0;
	virtual ~ControllerBase(){}
};

}

