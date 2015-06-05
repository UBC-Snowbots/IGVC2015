#pragma once

#include <jaus/mobility/drivers/LocalWaypointDriver.h>
#include <jaus/mobility/drivers/LocalWaypointListDriver.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include <jaus/core/management/Management.h>
#include "ros/ros.h"
#include "sb_msgs/MoveCommand.h"
#include <jaus/mobility/drivers/SetLocalWaypoint.h>

class LocalWaypointDriver: public JAUS::LocalWaypointDriver {
private:
    ros::Publisher pub;
    JAUS::LocalPoseSensor* localPose;
    JAUS::LocalWaypointListDriver* listDriver;
public:
    LocalWaypointDriver(const ros::Publisher& pub, JAUS::LocalPoseSensor* localPose, JAUS::LocalWaypointListDriver* listDriver): pub(pub), localPose(localPose), listDriver(listDriver) {}
    JAUS::Message* GenerateDriveCommand(const JAUS::Byte status) override;
    JAUS::Message* GenerateIdleDriveCommand(const JAUS::Byte status) const override;
    bool IsWaypointAchieved(const JAUS::LocalPose& currentPose,const JAUS::SetLocalWaypoint& desiredWaypoint) const override;
    void WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint) override;
    void setListDriver(JAUS::LocalWaypointListDriver* listDriver);
};

