#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>
#include <iostream>

#include <stdio.h>

using namespace std;

int main(int argc, char* argv[])
{
        if (argc < 3)
        {
                printf("Error: Arguement must include IP and Address \n");
        }

        char* ip_address = argv[1];
        char* comp_address = argv[2];

        cout << "ip_address: " << ip_address << " comp_address: " << comp_address << std::endl;
        
        JAUS::Component component;
        JAUS::Discovery* discoveryService = nullptr;
        JAUS::Transport* transportService = nullptr;

        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,"Snowbots");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");
        int comp_id = 1000;
        JAUS::Address componentID(comp_id,1,1);
        while(component.Initialize(componentID)==false){
                std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
                comp_id++;
                componentID(comp_id,1,1);
        }
        std::cout << "Success!" << std::endl;


        transportService = (JAUS::Transport*) component.GetService(JAUS::Transport::Name);
        transportService->LoadSettings("services.xml");

        const JAUS::Address comp_address_id = component.GetComponentID();
        if(!transportService->IsInitialized())
        {
                transportService->Initialize(comp_address_id);
        }
        
        // Create connection to OCP for the JAUS Interoperability Challenge using JUDP.
        transportService->AddNetworkConnection(JAUS::Address(8000, 1, 1), 
                            std::string(ip_address),
                            3794);

        JAUS::Management* managementService = nullptr;
        managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);
        
        JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
        while(true)
        {
                if(managementService->GetStatus() == JAUS::Management::Status::Shutdown)
                {
                        break;
                }
                if(JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 500)
                {
                        std::cout << "==================" << std::endl;
                        managementService->PrintStatus();
                        discoveryService->PrintStatus(); //std::cout << std::endl;
                        transportService->PrintStatus();
                        std::cout << std::endl;
                        displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
                }

                CxUtils::SleepMs(1);
        
        }

        component.Shutdown();
        return 0;
}

