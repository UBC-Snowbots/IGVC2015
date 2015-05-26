#pragma once

#include <jaus/mobility/drivers/LocalWaypointDriver.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include <jaus/core/management/Management.h>
#include "ros/ros.h"
#include "sb_msgs/MoveCommand.h"
#include <jaus/mobility/drivers/SetLocalWaypoint.h>

<<<<<<< HEAD
=======
#include "LocalWaypointListDriver.hpp"

>>>>>>> 92d6ab15b7d3a1dc6dc48ef49f720f32f9bcdf04
const double EqualityThreshold = 2; // change for actual sensible value (distance at which two waypoints should be considered equal)

class LocalWaypointDriver: public JAUS::LocalWaypointDriver {
private:
    ros::Publisher pub;
    JAUS::LocalPoseSensor* localPose;
<<<<<<< HEAD
    LocalWaypointListDriver* listDriver;
public:
    LocalWaypointDriver(const ros::Publisher& pub, JAUS::LocalPoseSensor* localPose, LocalWaypointListDriver* listDriver): pub(pub), localPose(localPose), listDriver(listDriver) {}
=======
    JAUS::LocalWaypointListDriver* listDriver;
public:
    LocalWaypointDriver(const ros::Publisher& pub, JAUS::LocalPoseSensor* localPose, JAUS::LocalWaypointListDriver* listDriver): pub(pub), localPose(localPose), listDriver(listDriver) {}
>>>>>>> 92d6ab15b7d3a1dc6dc48ef49f720f32f9bcdf04
    JAUS::Message* GenerateDriveCommand(const JAUS::Byte status) override;
    JAUS::Message* GenerateIdleDriveCommand(const JAUS::Byte status) const override;
    bool IsWaypointAchieved(const JAUS::LocalPose& currentPose,const JAUS::SetLocalWaypoint& desiredWaypoint) const override;
    void WaypointAchieved(const JAUS::SetLocalWaypoint& waypoint) override;
<<<<<<< HEAD
=======
    void setListDriver(JAUS::LocalWaypointListDriver* listDriver);
>>>>>>> 92d6ab15b7d3a1dc6dc48ef49f720f32f9bcdf04
};

