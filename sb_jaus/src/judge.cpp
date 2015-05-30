#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>

#include <jaus/mobility/sensors/QueryAccelerationState.h>
#include <jaus/mobility/sensors/ReportAccelerationState.h>

#include <jaus/mobility/sensors/QueryLocalPose.h>
#include <jaus/mobility/sensors/ReportLocalPose.h>
#include <jaus/mobility/sensors/SetLocalPose.h>

#include <jaus/mobility/sensors/QueryVelocityState.h>
#include <jaus/mobility/sensors/ReportVelocityState.h>
//Cannot set velocity in this kind of message lol

#include <jaus/mobility/sensors/QueryGlobalPose.h>
#include <jaus/mobility/sensors/ReportGlobalPose.h>
#include <jaus/mobility/sensors/SetGlobalPose.h>

#include <jaus/core/liveness/QueryHeartbeatPulse.h>
#include <jaus/core/liveness/ReportHeartbeatPulse.h>

#include <jaus/core/control/QueryControl.h>
#include <jaus/core/control/RequestControl.h>

#include <jaus/core/events/CreateEvent.h>
#include <jaus/core/events/QueryEvents.h>

#include <iostream>
#include <thread>
#include <unistd.h>
#include <chrono>

#include <iostream>
#include <limits>



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
    transportService->EnableDebugMessages(false);
    
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
    while(!component.Send(&id_query, &id_response));
    std::cout << "\033[4;31mID:\033[0m: " << id_response.GetIdentification() << std::endl;


    //Test to see if our bot can basic
    JAUS::QueryHeartbeatPulse heart_query(JAUS::Address(5000,1,1),component.GetComponentID());
    JAUS::ReportHeartbeatPulse heart_response;
    while(!component.Send(&heart_query, &heart_response));
    std::cout << "\033[4;31mHeart Response:\033[0m";
    heart_response.Print();
    
    //Status?
    std::cout << "\033[4;31mStatus Query:\033[0m";
    JAUS::QueryStatus statusQuery(JAUS::Address(5000,1,1), component.GetComponentID());
    JAUS::ReportStatus statusResponse;
    while(!component.Send(&statusQuery, &statusResponse));
    statusResponse.Print();
    
    std::cout << "\033[4;31mEvent Status Query:\033[0m";
    JAUS::CreateEvent event(JAUS::Address(5000,1,1), component.GetComponentID());
    event.SetQueryMessage(&statusQuery);
    while(!component.Send(&event, &statusResponse));
    statusResponse.Print();
    
    
    std::cout << "\033[4;31mControl Query:\033[0m" << std::endl;
    JAUS::QueryControl controlQuery(JAUS::Address(5000,1,1), component.GetComponentID());
    JAUS::ReportControl controlResponse;
    while(!component.Send(&controlQuery, &controlResponse));
    controlResponse.Print();

    //Time to find out if the auth be working
    std::cout << "\033[4;31mControl Attempt:\033[0m" << std::endl;
    JAUS::AccessControl* access_control = (JAUS::AccessControl*)component.GetService(JAUS::AccessControl::Name);
    access_control->EnableDebugMessages(true);
    access_control->SetAuthorityCode(255);
    
    while(!access_control->RequestComponentControl(JAUS::Address(5000,1,1)));
    
    JAUS::QueryAuthority auth_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportAuthority auth_response;
    while(!component.Send(&auth_query, &auth_response));
    auth_response.Print();
    
    
    
    //std::cout << "\033[4;31mControl Query:\033[0m" << std::endl;
    //component.Send((&controlQuery, &controlResponse));
    //controlResponse.PrintMessageBody();
    
    //Status?
    std::cout << "\033[4;31mStatus Query:\033[0m" << std::endl;
    while(!component.Send(&statusQuery, &statusResponse));
    statusResponse.Print();
    
    //Can we get control?
    //JAUS::RequestControl requestControl(JAUS::Address(5000,1,1), component.GetComponentID());
    //JAUS::ReportControl reportControl;
    //requestControl.SetAuthorityCode(0);
    
    //component.Send(&requestControl, &reportControl);
    
    //reportControl.PrintMessageBody();
    
    std::cout << "\033[4;31mAuthentication Query:\033[0m" << std::endl;
    JAUS::QueryAuthority queryAuth(JAUS::Address(5000,1,1), component.GetComponentID());
    JAUS::ReportAuthority reportAuth;
    
    while(!component.Send(&queryAuth, &reportAuth));
    reportAuth.Print();
    
    /*
    0 Velocity X
    1 Velocity Y
    2 Velocity Z
    3 Velocity RMS
    4 Roll Rate
    5 Pitch Rate
    6 Yaw Rate
    7 Rate RMS
    8 Time Stamp
     */

    //Now to see if our bot can strike a pose
    //Currently causing server side to report unsupported type (our system does not beleive that there is a GPS, but does have  a local pose?)
    JAUS::QueryGlobalPose global_pose_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    {
      typedef JAUS::QueryGlobalPose::PresenceVector JAUSPresenceVector;
      ushort presenceVector = 0; //does it work?
      presenceVector |= JAUSPresenceVector::Latitude;
      presenceVector |= JAUSPresenceVector::Longitude;
      presenceVector |= JAUSPresenceVector::Yaw;
    global_pose_query.SetPresenceVector(presenceVector);
    }
    JAUS::ReportGlobalPose global_pose_response;
    while(!component.Send(&global_pose_query, &global_pose_response));
    global_pose_response.PrintMessageBody();

    //How about we meet the locals?
    JAUS::QueryLocalPose local_pose_query(JAUS::Address(5000, 1, 1), component.GetComponentID());
    JAUS::ReportLocalPose local_pose_response;
    {
      typedef JAUS::QueryLocalPose::PresenceVector JAUSPresenceVector;
      ushort presenceVector = 0;
      presenceVector |= JAUSPresenceVector::X;
      presenceVector |= JAUSPresenceVector::Y;
      presenceVector |= JAUSPresenceVector::Yaw;
    local_pose_query.SetPresenceVector(presenceVector);
    }
    while(!component.Send(&local_pose_query, &local_pose_response));
    local_pose_response.PrintMessageBody();
        
    //How about we reset the locals?
    JAUS::SetLocalPose local_pose_set(JAUS::Address(5000, 1, 1), component.GetComponentID());
    
    //First we reset, so that the robot's refference point is wherever the hell it is
    JAUS::Point3D resetPosition(0,0,0);
    JAUS::Point3D resetOrientation(0,0,0); //Could elminate what is essentially a duplicate, but readability is nice
    
    local_pose_set.SetPose(resetPosition, resetOrientation); // 3rd param may be needed. IDK.
    
    //Two potential problems:
    //1. Does setting the pose actually make JAUS reply with the current pose?
    //2. Deos send NEED the address of a pointer? Can I not simply give the address of a 
    while(!component.Send(&local_pose_set, &local_pose_response));
    
    
    JAUS::Point3D additionToLocation(10,10,0); //X Y Z . How much is 10 X? Fuck if I know
    JAUS::Point3D additionToOrientation(0,0,10); //Roll Pitch Yaw. Also this robot identifies as a bottle of deoderant.
   
    local_pose_set.AddToPose(additionToLocation, additionToOrientation); //3rd param is time. unsure if needed.
    
    
    

    //Debug still live
    component.Shutdown();
    return 0;
}

