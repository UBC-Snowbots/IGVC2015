#pragma once

#include "ros/ros.h"
#include <geometry_msgs/Twist.h>

namespace ai{

class ControllerBase{
public:
	virtual int get_clock_speed() { return 10; }
	virtual geometry_msgs::Twist update() = 0;
	virtual ~ControllerBase(){}
};

}

