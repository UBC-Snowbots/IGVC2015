#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include <jaus/mobility/sensors/VelocityStateSensor.h>
#include "LocalWaypointDriver.hpp"
#include "LocalWaypointListDriver.hpp"
#include "LocalPoseSensorManager.hpp"
#include <iostream>
#include <ros/ros.h>

#include <cmath>

#include <sb_msgs/RobotState.h>
#include <sb_msgs/Waypoint.h>

const std::string MOVE_COMMAND_TOPIC = "move_command";

JAUS::LocalPoseSensor* localPoseSensor = nullptr;
JAUS::VelocityStateSensor* velocityStateSensor = nullptr;

double getStereoAngularVelocity(double l, double r){
	double diff = l-r;
	const double wheel_diameter = 0.62;
	double angle = asin(diff/wheel_diameter);
	return angle;
}

void onRobotStateChange(const sb_msgs::RobotState& state){
	JAUS::ReportLocalPose pose;
	pose.SetYaw(state.compass*CxUtils::PI/180);
	localPoseSensor->SetLocalPose(pose);
	
	double l = state.LeftVelo,r = state.RightVelo;
	
	JAUS::ReportVelocityState vstate;
	vstate.SetVelocityX((l + r)/2.0);
	vstate.SetYawRate(getStereoAngularVelocity(l,r));
	velocityStateSensor->SetVelocityState(vstate);
}

void onNewWaypoint(const sb_msgs::Waypoint& pos){
	JAUS::ReportLocalPose pose;
	pose.SetX(pos.lat);
	pose.SetY(pos.lon);
	localPoseSensor->SetLocalPose(pose);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "jaus");

    ros::NodeHandle self;

    JAUS::Component component;
    JAUS::Discovery* discoveryService = nullptr;
    JAUS::Transport* transportService = nullptr;
    localPoseSensor = new JAUS::LocalPoseSensor();
    velocityStateSensor = new JAUS::VelocityStateSensor();
    LocalWaypointDriver* localWaypointDriver = new LocalWaypointDriver(self.advertise<sb_msgs::MoveCommand>(MOVE_COMMAND_TOPIC,100),localPoseSensor,nullptr);
    LocalWaypointListDriver* localWaypointListDriver = new LocalWaypointListDriver(localWaypointDriver);
    localWaypointDriver->setListDriver(localWaypointListDriver);
    
    {
    	JAUS::ReportLocalPose localPose;
    	localPose.SetX(1.0);
    	localPose.SetY(1.0);
    	localPose.SetYaw(0.1);
    	localPoseSensor->SetLocalPose(localPose);
    }
    {
    	JAUS::ReportVelocityState state;
    	state.SetVelocityX(2);
    	state.SetYawRate(0.1);
    	velocityStateSensor->SetVelocityState(state);
    }

    self.subscribe("robot_state",100,onRobotStateChange);
    self.subscribe("GPS_COORD",100,onNewWaypoint);

    component.AddService(localPoseSensor);
    component.AddService(localWaypointDriver);
	component.AddService(velocityStateSensor);

    discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
    discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,"Snowbots");
    discoveryService->SetNodeIdentification("Main");
    discoveryService->SetComponentIdentification("Baseline");
    int comp_id = 5000;
    JAUS::Address componentID(comp_id,1,1);

    discoveryService->EnableDebugMessages(true);
    while(component.Initialize(componentID)==false) {
        std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
        comp_id++;
        componentID(comp_id,1,1);
    }
    std::cout << "Success!" << std::endl;

    transportService = (JAUS::Transport*) component.GetService(JAUS::Transport::Name);
    transportService->LoadSettings("services.xml");
	
    const JAUS::Address comp_address_id = component.GetComponentID();
    if(!transportService->IsInitialized()) {
        transportService->Initialize(comp_address_id);
    }

    JAUS::Management* managementService = nullptr;
    managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);

    JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        if(managementService->GetStatus() == JAUS::Management::Status::Shutdown) {
            break;
        }
        if(JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 500) {
            std::cout << "==================" << std::endl;
            managementService->PrintStatus();
            discoveryService->PrintStatus();
            transportService->PrintStatus();
            std::cout << std::endl;
            displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
    component.Shutdown();
    return 0;
}

