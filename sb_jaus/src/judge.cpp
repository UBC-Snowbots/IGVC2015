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

int main() {
    JAUS::Component component;
    JAUS::Discovery* discoveryService = nullptr;
    JAUS::Transport* transportService = nullptr;

    discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);

    discoveryService->EnableDebugMessages(false);

    discoveryService->SetSubsystemIdentification(JAUS::Subsystem::OtherSubsystem,"SnowBots_JudgeSim");
    discoveryService->SetNodeIdentification("Main");
    discoveryService->SetComponentIdentification("Baseline");
    
    transportService = (JAUS::Transport*)component.GetService(JAUS::Transport::Name);
    transportService->SetDisconnectTimeMs(10000);
    transportService->EnableDebugMessages(true);
    
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

    //Original, working code
    JAUS::QueryIdentification og_id_query(JAUS::Address(5000,1,1),component.GetComponentID());
    JAUS::ReportIdentification og_id_response;
    while(!component.Send(&og_id_query,&og_id_response,5000));
    std::cout << "ID: " << og_id_response.GetIdentification() << std::endl;

    
    JAUS::QueryIdentification id_query(JAUS::Address(5000,1,1),component.GetComponentID());
    JAUS::ReportIdentification id_response;
    component.Send(&id_query, &id_response);
    std::cout << "ID: " << id_response.GetIdentification() << std::endl;


    //Test to see if our bot can basic
    JAUS::QueryHeartbeatPulse heart_query(JAUS::Address(5000,1,1),component.GetComponentID());
    JAUS::ReportHeartbeatPulse heart_response;
    component.Send(&heart_query, &heart_response);
    heart_response.PrintMessageBody();
    

    //Time to find out if the auth be working
    JAUS::AccessControl* access_control = (JAUS::AccessControl*)component.GetService(JAUS::AccessControl::Name);
    access_control->SetAuthorityCode(255);
    JAUS::QueryAuthority auth_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportAuthority auth_response;
    component.Send(&auth_query, &auth_response);
    auth_response.PrintMessageBody();

    //Now to see if our bot can strike a pose
    JAUS::QueryGlobalPose global_pose_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportGlobalPose global_pose_response;
    component.Send(&global_pose_query, &global_pose_response);
    global_pose_response.PrintMessageBody();

    //How about we meet the locals?
    JAUS::QueryLocalPose local_pose_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportLocalPose local_pose_response;
    component.Send(&local_pose_query, &local_pose_response);
    local_pose_response.PrintMessageBody();
    
    //Debug still live
    component.Shutdown();
    return 0;
}

