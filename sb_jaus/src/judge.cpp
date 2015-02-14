#include <jaus/core/Component.h>
#include <iostream>
#include <thread>

const unsigned int SUBSYSTEM_TO_TEST = 100;

JAUS::Component component;

void run_testing(){
    std::cout << "jk not really - need to write this bit" << std::endl;
}

std::thread testing;

class DiscoveryCallback: public JAUS::Discovery::Callback{
	public:
	void ProcessMessage(const JAUS::Message* msg) override{
		JAUS::Discovery::Callback::ProcessMessage(msg);
		const JAUS::ReportIdentification* report = dynamic_cast<const JAUS::ReportIdentification*>(msg);
		if(report){
            std::string type;
			switch(report->GetType()){
				case JAUS::ReportIdentification::IdentificationType::Vehicle:
					type = "Vehicle";
					break;
				case JAUS::ReportIdentification::IdentificationType::OtherSubsystem:
					type = "Other Subsystem";
					break;
				case JAUS::ReportIdentification::IdentificationType::Node:
					type = "Node";
					break;
				case JAUS::ReportIdentification::IdentificationType::Component:
					type = "Component";
					break;
				default:
					type = "Something";
			}
			std::cout << "Found " << type << ": " << report->GetIdentification() << std::endl;
			
			if(report->GetSourceID().mSubsystem == SUBSYSTEM_TO_TEST){
				std::cout << "This is the test subsystem - start testing thread" << std::endl;

                testing = std::thread(run_testing);
			}
		}
	}
};

int main(){
        JAUS::Discovery* discoveryService = nullptr;
        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        
        discoveryService->EnableDebugMessages(true);

        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::OtherSubsystem,"SnowBots_JudgeSim");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");
        
        if(!discoveryService->SetDiscoveryFrequency(0.2)){
            std::cout << "Could not set discovery frequency" << std::endl;
            return -1;
        }
        discoveryService->RegisterCallback(new DiscoveryCallback());
        discoveryService->DiscoverSubsystems(true);
        
        
        JAUS::Address componentID(200,1,1);
        if(component.Initialize(componentID)==false){
                std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
                return 0;
        }
        if(discoveryService->is)
        std::cout << "Initialisation successful!" << std::endl;
        
        JAUS::QueryIdentification query_id(JAUS::Address(JAUS::Address::GlobalBroadcast), componentID);
        
        if(!component.Send(&query_id, 2)){
            std::cout << "could not send discovery message..." << std::endl;
            return -1;
        }
        
        while(true){
                JAUS::Management* managementService = nullptr;
                managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);
                if(managementService->GetStatus() == JAUS::Management::Status::Shutdown){
                        break;
                }
                CxUtils::SleepMs(1);
        }
        
        component.Shutdown();
        return 0;
}

