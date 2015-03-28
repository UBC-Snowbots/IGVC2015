#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>
#include <iostream>
#include <thread>

const unsigned int SUBSYSTEM_TO_TEST = 100;

JAUS::Component component;

int main(){
        JAUS::Component component;
        JAUS::Discovery* discoveryService = nullptr;
        JAUS::Transport* transportService = nullptr;

        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        
        discoveryService->EnableDebugMessages(true);

        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::OtherSubsystem,"SnowBots_JudgeSim");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");
        
        // Judge initializes @ id = 1000
        int comp_id = 1000;
        JAUS::Address componentID(comp_id,1,1);
        // hacks if id = 1000 is already taken
        while(component.Initialize(componentID)==false){
                std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
                comp_id++;
                componentID(comp_id,1,1);
        }


        if(discoveryService->IsEnabled())
        {
        	std::cout << "Initialisation successful!" << std::endl;
        }else
        {
        	std::cerr << "Discovery disabled. Abort." << std::endl;
        	return 1;
        }

        // Loading transport settings
        transportService = (JAUS::Transport*) component.GetService(JAUS::Transport::Name);
        transportService->LoadSettings("services.xml");
        // if failed to initialize at first 
        if(!transportService->IsInitialized())
        {
            // re-initialize
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

