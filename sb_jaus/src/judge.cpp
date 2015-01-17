#include <jaus/core/Component.h>
#include <iostream>

int main(){
        JAUS::Component component;
        JAUS::Discovery* discoveryService = nullptr;
        discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);
        
        discoveryService->SetSubsystemIdentification(JAUS::Subsystem::Vehicle,"SnowBots_JudgeSim");
        discoveryService->SetNodeIdentification("Main");
        discoveryService->SetComponentIdentification("Baseline");
        
        JAUS::Address componentID(2000,1,1);
        if(component.Initialize(componentID)==false){
                std::cout << "Failed to initialize [" << componentID.ToString() << std::endl;
                return 0;
        }
        std::cout << "Success!" << std::endl;
        
        JAUS::QueryIdentification id_query(JAUS::Address(1000,1,1),component.GetComponentID());
        JAUS::ReportIdentification id_response;
        while(!component.Send(&id_query,&id_response,5000));
        std::cout << "ID: " << id_response.GetIdentification() << std::endl;
        
        component.Shutdown();
        return 0;
}

