#include <jaus/core/Component.h>
#include <iostream>

int main(){
        JAUS::Component component;
        JAUS::Discovery* discoveryService = nullptr;
        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        
        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,"Snowbots");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");
        
        JAUS::Address componentID(1000,1,1);
        if(component.Initialize(componentID)==false){
                std::cout << "Failed to initialize [" << componentID.ToString() << std::endl;
                return 0;
        }
        std::cout << "Success!" << std::endl;
        
        JAUS::Time::Stamp displayStatusTimeMs = JAUS::Time::GetUtcTimeMs();
        while(true){
                JAUS::Management* managementService = nullptr;
                managementService = (JAUS::Management*)component.GetService(JAUS::Management::Name);
                if(managementService->GetStatus() == JAUS::Management::Status::Shutdown){
                        break;
                }
                if(JAUS::Time::GetUtcTimeMs() - displayStatusTimeMs > 500){
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

