#include "LocalWaypointListDriver.hpp"

void LocalWaypointListDriver::ExecuteList(const double speedMetersPerSecond){
	std::vector<JAUS::SetLocalWaypoint> wpts = std::move(GetWaypointList());
    if(wpts.size()>0){
    	driver->SetLocalWaypoint(&wpts[0]);
    	driver->SetDesiredTravelSpeed(speedMetersPerSecond);
    }
}

