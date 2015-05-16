#include <jaus/mobility/drivers/LocalWaypointDriver.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include <jaus/core/management/Management.h>
#include "ros/ros.h"
#include "sb_msgs/MoveCommand.h"
#include <jaus/mobility/drivers/SetLocalWaypoint.h>

const double EqualityThreshold = 2; // change for actual sensible value (distance at which two waypoints should be considered equal)

class LocalWaypointDriver: public JAUS::LocalWaypointDriver {
private:
    ros::Publisher pub;
    JAUS::LocalPoseSensor* localPose;
public:
    LocalWaypointDriver(const ros::Publisher& pub, JAUS::LocalPoseSensor* localPose): pub(pub), localPose(localPose) {}
    JAUS::Message* GenerateDriveCommand(const JAUS::Byte status) override {
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
    JAUS::Message* GenerateIdleDriveCommand(const JAUS::Byte status) const override {
        sb_msgs::MoveCommand com;
        com.spd = 0;
        pub.publish(com);
        return NULL;
    }
    bool IsWaypointAchieved(const JAUS::LocalPose& currentPose,const JAUS::SetLocalWaypoint& desiredWaypoint) const override {
        return sqrt(pow(currentPose.GetX()-desiredWaypoint.GetX(),2) + pow(currentPose.GetY()-desiredWaypoint.GetY(),2)) < EqualityThreshold;
    }
    void WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint) override {
        //do nothing :)
    }
};

