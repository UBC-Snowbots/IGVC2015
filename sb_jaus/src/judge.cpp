#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>

#include <jaus/mobility/sensors/QueryAccelerationState.h>
#include <jaus/mobility/sensors/ReportAccelerationState.h>

#include <jaus/mobility/sensors/QueryLocalPose.h>
#include <jaus/mobility/sensors/ReportLocalPose.h>

#include <jaus/mobility/sensors/QueryVelocityState.h>
#include <jaus/mobility/sensors/ReportVelocityState.h>

#include <jaus/mobility/sensors/QueryGlobalPose.h>
#include <jaus/mobility/sensors/ReportGlobalPose.h>

#include <jaus/core/liveness/QueryHeartbeatPulse.h>
#include <jaus/core/liveness/ReportHeartbeatPulse.h>


#include <iostream>
#include <thread>
#include <unistd.h>
#include <chrono>

const unsigned int SUBSYSTEM_TO_TEST = 100;

JAUS::Component component;

void run_testing() {
    std::cout << "jk not really - need to write this bit" << std::endl;
}

std::thread testing;

void communications_loop(JAUS::Component &comp, JAUS::Message &query, JAUS::Message &response){
  unsigned int i = 0;
    while(!component.Send(query,response,5000)){
      ++i;
      std::cout << "...";
      usleep(1000);
      std::cout << "\b\b\b\033[K";
    }
    std::cout << "Waited: " << i*1000 << "ms" << std::endl;
}


class DiscoveryCallback: public JAUS::Discovery::Callback {
public:
    void ProcessMessage(const JAUS::Message* msg) override {
        JAUS::Discovery::Callback::ProcessMessage(msg);
        const JAUS::ReportIdentification* report = dynamic_cast<const JAUS::ReportIdentification*>(msg);
        if(report) {
            std::string type;
            switch(report->GetType()) {
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

            if(report->GetSourceID().mSubsystem == SUBSYSTEM_TO_TEST) {
                std::cout << "This is the test subsystem - start testing thread" << std::endl;

                testing = std::thread(run_testing);
            }
        }
    }
};

int main() {
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
    while(component.Initialize(componentID)==false) {
        std::cout << "Failed to initialize [" << componentID.ToString() << "]" << std::endl;
        comp_id++;
        componentID(comp_id,1,1);
    }
    if(discoveryService->IsEnabled()) {
        std::cout << "Initialisation successful!" << std::endl;
    } else {
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
    communications_loop(component, id_query, id_response);
    std::cout << "ID: " << id_response.GetIdentification() << std::endl;
    
    //Test to see if our bot can basic
    JAUS::QueryHeartbeatPulse heart_query(JAUS::Address(5000,1,1),component.GetComponentID());
    JAUS::ReportHeartbeatPulse heart_response;
    communications_loop(component, heart_query, heart_response);
    heart_response.PrintMessageBody();

    //Time to find out if the auth be working
    JAUS::QueryAuthority auth_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportAuthority auth_response;
    communications_loop(component, auth_query, auth_response);
    auth_response.PrintMessageBody();
    
    //Now to see if our bot can strike a pose
    JAUS::QueryGlobalPose global_pose_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportGlobalPose global_pose_response;
    communications_loop(component, global_pose_query, global_pose_response);
    global_pose_response.PrintMessageBody();
    
    //How about we meet the locals?
    JAUS::QueryLocalPose local_pose_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportLocalPose local_pose_response;
    communications_loop(component, local_pose_query, local_pose_response);
    local_pose_response.PrintMessageBody();
    
    
    
    //Debug still live
    component.Shutdown();
    return 0;
}

