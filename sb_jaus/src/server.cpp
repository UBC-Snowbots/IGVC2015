#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>
#include <iostream>

int main()
{
        JAUS::Component component;
        JAUS::Discovery* discoveryService = nullptr;
        JAUS::Transport* transportService = nullptr;

        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,"Snowbots");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");

        JAUS::Address componentID(7000,1,1);
        if(component.Initialize(componentID)==false)
        {
                std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
                return 0;
        }
        std::cout << "Success!" << std::endl;

        transportService = (JAUS::Transport*) component.GetService(JAUS::Transport::Name);
        transportService->LoadSettings("services.xml");
        if(!transportService->IsInitialized())
        {
                const JAUS::Address comp_address_id = component.GetComponentID();
                transportService->Initialize(comp_address_id);
        }

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
                        std::cout << std::endl;
                        displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
                }

                CxUtils::SleepMs(1);
        
        }
        component.Shutdown();
        return 0;
}

