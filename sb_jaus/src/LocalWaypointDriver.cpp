#include "LocalWaypointDriver.hpp"

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
    return sqrt(pow(currentPose.GetX()-desiredWaypoint.GetX(),2) + pow(currentPose.GetY()-desiredWaypoint.GetY(),2)) < 1;
}
void LocalWaypointDriver::WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint){
    std::vector<JAUS::SetLocalWaypoint> wpts = std::move(listDriver->GetWaypointList());
    if(wpts.size()>0){
    	SetLocalWaypoint(&wpts[0]);
    }
}
void LocalWaypointDriver::setListDriver(JAUS::LocalWaypointListDriver* listDriver){
	this->listDriver = listDriver;
}

