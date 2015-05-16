#include "LocalPoseSensor.hpp"

bool LocalPoseSensor::onPoseChange(const std_msgs::String& msg){
	// stub
}
LocalPoseSensor::LocalPoseSensor(JAUS::LocalPoaseSensor* sensor): sensor(sensor){
	ros::NodeHandle nh;
	sub = nh.subscribe<std_msgs::String>(POSE_TOPIC,100,&LocalPoseSensor::onPoseChange,this);
}

