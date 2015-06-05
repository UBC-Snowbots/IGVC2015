#include "LocalWaypointDriver.hpp"

#include <ros/ros.h>
#include <sb_gps/Gps_Service.h>

JAUS::Message* LocalWaypointDriver::GenerateDriveCommand(const JAUS::Byte status) {
    if(status==JAUS::Management::Status::Ready || status == JAUS::Management::Status::Standby) {
        sb_msgs::MoveCommand com;
        JAUS::SetLocalWaypoint wp = GetLocalWaypoint();
        JAUS::GlobalPose gp = localPose->GetLocalPoseReference();
        com.lat = wp.GetX() + gp.GetLongitude();
        com.lon = wp.GetY() + gp.GetLatitude();
        com.spd = GetDesiredTravelSpeed().GetSpeed();
        pub.publish(com);
        return NULL;
    }
}
JAUS::Message* LocalWaypointDriver::GenerateIdleDriveCommand(const JAUS::Byte status) const {
    sb_msgs::MoveCommand com;
    com.spd = 0;
    pub.publish(com);
    return NULL;
}
bool LocalWaypointDriver::IsWaypointAchieved(const JAUS::LocalPose& currentPose,const JAUS::SetLocalWaypoint& desiredWaypoint) const{
    	sb_gps::Gps_Service srv;
    	srv.request.lat1 = currentPose.GetX();
    	srv.request.lon1 = currentPose.GetY();
    	srv.request.lat2 = desiredWaypoint.GetX();
    	srv.request.lon2 = desiredWaypoint.GetY();
    	if(ros::service::call("GPS_SERVICE",srv)){
    		if(srv.response.distance < 11) return true;
    	}
    	return false;
}
void LocalWaypointDriver::WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint){
    std::vector<JAUS::SetLocalWaypoint> wpts = listDriver->GetWaypointList();
    if(wpts.size()>0){
    	SetLocalWaypoint(&wpts[0]);
    }
}
void LocalWaypointDriver::setListDriver(JAUS::LocalWaypointListDriver* listDriver){
	this->listDriver = listDriver;
}

