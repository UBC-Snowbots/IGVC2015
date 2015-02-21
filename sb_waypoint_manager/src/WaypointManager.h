#pragma once

#include "ros/ros.h"
#include "sb_msgs/Waypoint.h"
#include "sb_msgs/WaypointList.h"
#include <vector>

namespace sb_waypoint_manager{

class WaypointManager{
private:
	ros::ServiceClient gps_client;
	ros::Subscriber waypoint_list_subscriber;
	
	std::vector<sb_msgs::Waypoint> waypointList;
	
	void process_waypoint_list(const sb_msgs::WaypointList& list);
public:
	WaypointManager(ros::NodeHandle);
	~WaypointManager();
	
	void stop();
	void updateGPS();
	
};

}

