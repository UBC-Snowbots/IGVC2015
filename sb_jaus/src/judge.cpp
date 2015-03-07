#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>
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
        JAUS::Component component;
        JAUS::Discovery* discoveryService = nullptr;
        JAUS::Transport* transportService = nullptr;

        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        
        discoveryService->EnableDebugMessages(true);

        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::OtherSubsystem,"SnowBots_JudgeSim");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");
        int comp_id = 1000;
        JAUS::Address componentID(comp_id,1,1);
        while(component.Initialize(componentID)==false){
                std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
                comp_id++;
                componentID(comp_id,1,1);
        }
        if(discoveryService->IsEnabled()){
        	std::cout << "Initialisation successful!" << std::endl;
        }else{
        	std::cerr << "Discovery disabled. Abort." << std::endl;
        	return 1;
        }

        transportService = (JAUS::Transport*) component.GetService(JAUS::Transport::Name);
        transportService->LoadSettings("services.xml");
        if(!transportService->IsInitialized())
        {
                const JAUS::Address comp_address_id = component.GetComponentID();
                transportService->Initialize(comp_address_id);
        }
        
	JAUS::QueryIdentification id_query(JAUS::Address(5000,1,1),component.GetComponentID());
        JAUS::ReportIdentification id_response;
	while(!component.Send(&id_query,&id_response,5000));
        std::cout << "ID: " << id_response.GetIdentification() << std::endl;
        
        component.Shutdown();
        return 0;
}

