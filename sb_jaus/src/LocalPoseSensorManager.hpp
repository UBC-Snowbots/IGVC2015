#pragma once

#include <ros/ros.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include "std_msgs/String.h"

const std::string POSE_TOPIC = "local_pose";

class LocalPoseSensorManager{
	private:
	JAUS::LocalPoseSensor* sensor;
	ros::Subscriber sub;
	public:
	void onPoseChange(std_msgs::String msg);
	LocalPoseSensorManager(ros::NodeHandle& nh, JAUS::LocalPoseSensor* sensor);
};

