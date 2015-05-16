#include <jaus/mobility/drivers/LocalWaypointDriver.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include "ros/ros.h"
#include "sb_msgs/MoveCommand.h"
#include <jaus/mobility/drivers/SetLocalWaypoint.h>

const double EqualityThreshold = 2; // change for actual sensible value (distance at which two waypoints should be considered equal)

class LocalWaypointDriver: public JAUS::LocalWaypointDriver{
	private:
	ros::Publisher pub;
	JAUS::LocalPoseSensor* localPose;
	public:
	LocalWaypointDriver(ros::Publisher& pub, JAUS::LocalPoseSensor* localPose): pub(pub), localPose(localPose) {}
	Message* GenerateDriveCommand(const Byte status){
		sb_msgs::MoveCommand com;
		JAUS::SetLocalWaypoint wp = GetLocalWaypoint();
		wp.
		JAUS::SetGlobalPose gp = localPose->GetLocalPoseReference();
		com.lat = wp.getX() + gp.getLongitude();
		com.lon = wp.getY() + gp.getLatitutde();
		com.spd = GetDesiredTravelSpeed();
		pub.publish(com);
		return NULL;
	}
	Message* GenerateIdleDriveCommand(const Byte status) const{
		sb_msgs::MoveCommand com;
		com.spd = 0;
		pub.publish(com);
		return NULL;
	}
	bool IsWaypointAchieved(const LocalPose& currentPose,const JAUS::SetLocalWaypoint& desiredWaypoint) const{
		return sqrt(pow(currentPose.getLatitude()-desiredWaypoint.getX(),2) + pow(currentPose.getLongitude()-desiredWaypoint.getY(),2)) < EqualityThreshold;
	}
	void WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint){
		//do nothing :)
	}
};

