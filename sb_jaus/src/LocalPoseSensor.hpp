#pragma once

#include <ros/ros.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include "std_msgs/String.h"

const std::string POSE_TOPIC = "no idea";

class LocalPoseSensor{
	private:
	JAUS::LocalPoseSensor* sensor;
	ros::Subscriber sub;
	public:
	bool onPoseChange(const std_msgs::String& msg);
	LocalPoseSensor(JAUS::LocalPoaseSensor* sensor);
};

