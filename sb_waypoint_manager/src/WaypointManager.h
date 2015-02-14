#pragma once

#include "ros/ros.h"
#include "sb_msgs/Waypoint.h"
#include <queue>

namespace sb_waypoint_manager{

class WaypointManager{
private:
	ros::ServiceClient client;
	std::queue<sb_msgs::Waypoint> waypointList;
public:
	WaypointManager(ros::NodeHandle);
	~WaypointManager();
	
	void loadFromFile(const char* fname);
	
	void updateGPS();
	
};

}

