#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>
#include <jaus/mobility/sensors/LocalPoseSensor.h>
#include "WaypointDriver.hpp"
#include "LocalPoseSensor.hpp"
#include <iostream>
#include <ros/ros.h>

const std::string MOVE_COMMAND_TOPIC = "move_command";

JAUS::LocalPoseSensor* localPoseSensor = nullptr;

int main(int argc, char* argv[]){
	ros::init(argc, argv, "jaus");
	
	ros::NodeHandle self;
	
	JAUS::Component component;
	JAUS::Discovery* discoveryService = nullptr;
	JAUS::Transport* transportService = nullptr;
	localPoseSensor = new JAUS::LocalPoseSensor();
	LocalWaypointDriver* localWaypointDriver = new LocalWaypointDriver(self.advertise<sb_msgs::MoveCommand>(MOVE_COMMAND_TOPIC,100),localPoseSensor);
	component.AddService(localPoseSensor);
	component.AddService(localWaypointDriver);
	LocalPoseSensor manager(localPoseSensor);
	
	discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
	discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,"Snowbots");
	discoveryService->SetNodeIdentification("Main");
	discoveryService->SetComponentIdentification("Baseline");
	int comp_id = 5000;
	JAUS::Address componentID(comp_id,1,1);
	
	discoveryService->EnableDebugMessages(true);
	while(component.Initialize(componentID)==false){
		std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
		comp_id++;
		componentID(comp_id,1,1);
	}
	std::cout << "Success!" << std::endl;
	
	transportService = (JAUS::Transport*) component.GetService(JAUS::Transport::Name);
	transportService->LoadSettings("services.xml");
	
	const JAUS::Address comp_address_id = component.GetComponentID();
	if(!transportService->IsInitialized()){
		transportService->Initialize(comp_address_id);
	}
	
	JAUS::Management* managementService = nullptr;
	managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);
	        
	JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
	ros::Rate loop_rate(10);
	
	while(ros::ok()){
		if(managementService->GetStatus() == JAUS::Management::Status::Shutdown){
			break;
		}
		if(JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 500){
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

