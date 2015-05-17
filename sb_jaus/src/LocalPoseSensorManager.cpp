#include "LocalPoseSensorManager.hpp"

bool LocalPoseSensorManager::onPoseChange(const std_msgs::String& msg) {

    JAUS::SetLocalPose localPose;

    //X
    localPose.SetX(1.0);

    //Y
    localPose.SetY(1.0);

    //Z lol nope z can fuck off
    //localPose.SetZ(1.0);

    //Pitch
    localPose.SetPitch(90.0);

    //Yaw

    //Roll

    sensor->SetLocalPose(localPose);

}
LocalPoseSensorManager::LocalPoseSensorManager(JAUS::LocalPoseSensor* sensor): sensor(sensor) {
    ros::NodeHandle nh;
    sub = nh.subscribe<std_msgs::String>(POSE_TOPIC,100,&LocalPoseSensorManager::onPoseChange,this);
}

