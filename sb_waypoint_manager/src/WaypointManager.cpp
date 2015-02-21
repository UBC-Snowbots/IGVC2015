#include "WaypointManager.h"
#include "sb_gps/GotoWaypoint.h"

namespace sb_waypoint_manager{

WaypointManager::WaypointManager(ros::NodeHandle node){
	gps_client = node.serviceClient<sb_gps::GotoWaypoint>("goto_waypoint");
	waypoint_list_subscriber = node.subscribe(sb_msgs::WaypointList::WAYPOINT_LIST,1,&WaypointManager::process_waypoint_list, this);
}
WaypointManager::~WaypointManager() {}

void WaypointManager::process_waypoint_list(const sb_msgs::WaypointList& list){
	waypointList = list.waypoints;
}

void WaypointManager::stop(){
	waypointList.clear();
	sb_gps::GotoWaypoint srv;
	srv.request.move = false;
	if(gps_client.call(srv)){
		//nothing to do here
	}else{
		ROS_ERROR("Failed to call service sb_gps::GotoWaypoint");
	}
}

void WaypointManager::updateGPS(){
	if(waypointList.empty()) return;
	
	sb_gps::GotoWaypoint srv;
	srv.request.location = waypointList.front();
	srv.request.move = true;
	if(gps_client.call(srv)){
		if(srv.response.at_location){
			waypointList.erase(waypointList.begin());
		}
	}else{
		ROS_ERROR("Failed to call service sb_gps::GotoWaypoint");
	}
}

}

