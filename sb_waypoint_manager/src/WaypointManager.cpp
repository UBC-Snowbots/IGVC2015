#include "WaypointManager.h"
#include "sb_gps/GotoWaypoint.h"

namespace sb_waypoint_manager{

WaypointManager::WaypointManager(ros::NodeHandle node):
client(node.serviceClient<sb_gps::GotoWaypoint>("sb_gps"))
{
	
}
WaypointManager::~WaypointManager() {}

void WaypointManager::loadFromFile(const char* fname){
	
}

void WaypointManager::updateGPS(){
	if(waypointList.empty()) return;
	
	sb_gps::GotoWaypoint srv;
	srv.request.location = waypointList.front();
	if(client.call(srv)){
		if(srv.response.at_location){
			waypointList.pop();
		}
	}else{
		ROS_ERROR("Failed to call service sb_gps::GotoWaypoint");
	}
}

}

