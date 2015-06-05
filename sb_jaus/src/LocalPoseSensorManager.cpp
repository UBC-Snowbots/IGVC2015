#include "LocalPoseSensorManager.hpp"
#include <cxutils/math/Point3D.h>

#include <cmath>

void LocalPoseSensorManager::onPoseChange(std_msgs::String msg) {

    JAUS::SetLocalPose localPose;

    //X
    localPose.SetX(1.0);

    //Y
    localPose.SetY(1.0);

    //Z lol nope z can fuck off
    //localPose.SetZ(1.0);

    //Pitch
    localPose.SetPitch(JAUS::SetLocalPose::Limits::MaxAngle/2);

    //Yaw

    //Roll

    sensor->SetLocalPose(localPose);

}
LocalPoseSensorManager::LocalPoseSensorManager(ros::NodeHandle& nh, JAUS::LocalPoseSensor* sensor): sensor(sensor) {
    sub = nh.subscribe<std_msgs::String>(POSE_TOPIC,100,&LocalPoseSensorManager::onPoseChange,this);
    JAUS::ReportLocalPose localPose;

    //X
    localPose.SetX(1.0);

    //Y
    localPose.SetY(1.0);

    //Pitch
    localPose.SetPitch(1);

    sensor->SetLocalPose(localPose);
}

