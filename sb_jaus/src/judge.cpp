#include <jaus/core/Component.h>
#include <jaus/core/transport/Transport.h>

#include <jaus/mobility/sensors/QueryAccelerationState.h>
#include <jaus/mobility/sensors/ReportAccelerationState.h>

#include <jaus/mobility/sensors/QueryLocalPose.h>
#include <jaus/mobility/sensors/ReportLocalPose.h>
#include <jaus/mobility/sensors/SetLocalPose.h>

#include <jaus/mobility/sensors/QueryVelocityState.h>
#include <jaus/mobility/sensors/ReportVelocityState.h>
#include <jaus/mobility/sensors/QueryGlobalPose.h>
#include <jaus/mobility/sensors/ReportGlobalPose.h>

#include <jaus/core/liveness/QueryHeartbeatPulse.h>
#include <jaus/core/liveness/ReportHeartbeatPulse.h>

#include <jaus/core/control/QueryControl.h>
#include <jaus/core/control/RequestControl.h>

#include <jaus/core/events/CreateEvent.h>
#include <jaus/core/events/QueryEvents.h>
#include <jaus/core/events/ReportEvents.h>
#include <jaus/core/events/UpdateEvent.h>
#include <jaus/core/events/ConfirmEventRequest.h>

#include <jaus/mobility/drivers/SetLocalWaypoint.h>

#include <iostream>
#include <thread>
#include <unistd.h>
#include <chrono>

#include <iostream>
#include <limits>



const unsigned int SUBSYSTEM_TO_TEST = 100;

class TestSubscriber : public JAUS::Events::Child {
    JAUS::Address targetComponent;
    //Constructor
public:
    TestSubscriber() : JAUS::Events::Child(JAUS::Service::ID("TestSubscriber"), // ID of my service.
                JAUS::Service::ID(JAUS::Events::Name))  // ID of the service I inherit from (events)
    {
        targetComponent(5000, 1, 1); //Component might be on 5000,1,2? or 3? or 100? IDK
    }
    
    ~TestSubscriber(){
      //Much deletion
    }

    bool IsDiscoverable() const {
        return false;
    }

    JAUS::Message* CreateMessage(const JAUS::UShort messageCode) const
    {
        JAUS::Message* message = NULL;
        switch(messageCode)
        {
        case JAUS::REPORT_LOCAL_POSE:
            message = new JAUS::ReportLocalPose;
            break;
        case JAUS::QUERY_LOCAL_POSE:
            message = new JAUS::QueryLocalPose;
            break;
        default:
            message = NULL;
            break;
        }
        return message;
    }

    void Receive(const JAUS::Message* message)
    {
        //Type cast the message and use the data
        switch(message->GetMessageCode())
        {
        case JAUS::REPORT_LOCAL_POSE:
        {
            const JAUS::ReportLocalPose* report =
                dynamic_cast<const JAUS::ReportLocalPose*>(message);
            if(report)
            {
                std::cout << "Rceived Local Pose Report message: ";
		message->Print();
		std::cout << std::endl;
            }
        }
        break;
        case JAUS::QUERY_LOCAL_POSE:
            //We don't generate local poses. The local pose query can fuck off
            break;
        default:
            break;
        }
    }

    bool CreateSubscriptions()
    {
        if(EventsService()->HaveSubscription(JAUS::REPORT_LOCAL_POSE,
                                             targetComponent) == false) {
            //Setup the query for the local pose
            JAUS::QueryLocalPose query;

            //Get all the data it will send
            query.SetPresenceVector(query.GetPresenceVectorMask());
            if(EventsService()->RequestEveryChangeEvent(targetComponent, &query)) {
                return true;
            }
        }
        return false;
    }

    bool GenerateEvent(const JAUS::Events::Subscription& info) const override
    {
      return false;
    }
    
    bool IsEventSupported(const JAUS::Events::Type type,
                                 const double requestedPeriodicRate,
                                 const JAUS::Message* queryMessage,
                                 double& confirmedPeriodicRate,
                                 std::string& errorMessage) const override{

        //We don;t support any event generation
        return false;
    }
};

int main() {


    JAUS::Component component;
    JAUS::Discovery* discoveryService = nullptr;
    JAUS::Transport* transportService = nullptr;
    
    TestSubscriber* service = new TestSubscriber();
    component.AddService(service);

    discoveryService = (JAUS::Discovery*)component.GetService(JAUS::Discovery::Name);

    discoveryService->EnableDebugMessages(false);

    discoveryService->SetSubsystemIdentification(JAUS::Subsystem::OtherSubsystem,"SnowBots_JudgeSim");
    discoveryService->SetNodeIdentification("Main");
    discoveryService->SetComponentIdentification("Baseline");
    discoveryService->SetSubsystemsToDiscover(std::set<JAUS::UShort>());

    transportService = (JAUS::Transport*)component.GetService(JAUS::Transport::Name);
    transportService->SetDisconnectTimeMs(10000);
    transportService->EnableDebugMessages(false);


    JAUS::AccessControl* accessControl = (JAUS::AccessControl*)component.GetService(JAUS::AccessControl::Name);
    accessControl->EnableDebugMessages(false);
    accessControl->SetAuthorityCode(10);

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
    
    bool debug = false;

    //******************************************************************************************************
    //now we attempt to subscribe
    std::cout << "Attempt to subsribe commented out..." << std::endl;/*
    while(service->CreateSubscriptions() == false){
      
    }
      */
    while(true) {
        int choice;
        std::cout << "\033[5;33m Choose what to do: " << std::endl;
        std::cout << " 0 : EXIT" << std::endl;
        std::cout << " 1 : Toggle Debug " << std::endl;
        std::cout << " 2 : Query ID and Services" << std::endl;
        std::cout << " 3 : Query Heartbeat" << std::endl;
        std::cout << " 4 : Query Status " << std::endl;
        std::cout << " 5 : Query Control " << std::endl;
        std::cout << " 6 : Query Events " << std::endl;
        std::cout << " 7 : Query Local Pose " << std::endl;
        std::cout << " 8 : Query Global Pose " << std::endl;
        std::cout << " 9 : Take Control of Robot " << std::endl;
        std::cout << " 10: Set Local Pose " << std::endl;

        std::cout << "100: Shutdown" << std::endl;
        std::cout << "101: Set Emergency" << std::endl;
        std::cout << "102: Clear Emergency" << std::endl;
        std::cout << "\033[0m";
        std::cin >> choice;

        switch(choice) {
        case 0: {
            //QQ
            std::cout << "Exiting" << std::endl;
            component.Shutdown();
            exit(0);
        }
        case 1: {
            //Toggle Debug
            debug = !debug;
            discoveryService->EnableDebugMessages(debug);
            transportService->EnableDebugMessages(debug);
            accessControl->EnableDebugMessages(debug);
            break;
        }
        case 2: {
            //Query ID and services because we can
            std::cout << "\033[4;31mID:\033[0m" << std::endl;
            JAUS::QueryIdentification id_query(JAUS::Address(0xffff, 0xff, 0xff),component.GetComponentID());
            //JAUS::QueryIdentification id_query(JAUS::Address(103,1,1),component.GetComponentID());
            id_query.SetQueryType(JAUS::QueryIdentification::Type::ComponentIdentification);
            JAUS::ReportIdentification id_response;
            std::cout << "Return: " << component.Send(&id_query, &id_response) << std::endl;
            std::cout << id_response.GetIdentification() << std::endl;
	    
	    JAUS::QueryServices serviceQuery(JAUS::Address(0xffff,0xff,0xff), component.GetComponentID());
	    //JAUS::QueryServices serviceQuery(JAUS::Address(103, 1, 1), component.GetComponentID());
	    JAUS::ReportServices serviceReport;
	    std::cout << "\033[4;31mWith services:\033[0m" << std::endl;
	    std::cout << "Return: " << component.Send(&serviceQuery, &serviceReport) << std::endl;
	    serviceReport.Print();
            break;
        }
        case 3: {
            //Query Heartbeat
            JAUS::QueryHeartbeatPulse heart_query(JAUS::Address(5000,1,1),component.GetComponentID());
            JAUS::ReportHeartbeatPulse heart_response;
            std::cout << "Return: " << component.Send(&heart_query, &heart_response) << std::endl;
            std::cout << "\033[4;31mHeart Response:\033[0m";
            heart_response.GetMessageCodeOfResponse();
            break;
        }
        case 4: {
            //Query Status
            std::cout << "\033[4;31mEvent Status Query:\033[0m";
            JAUS::QueryStatus statusQuery(JAUS::Address(5000,1,1), component.GetComponentID());
            JAUS::ReportStatus statusResponse;
            JAUS::CreateEvent event(JAUS::Address(5000,1,1), component.GetComponentID());
            event.SetQueryMessage(&statusQuery);
            event.SetRequestID(1);
            std::cout << "Return: " << component.Send(&event, &statusResponse) << std::endl;
            statusResponse.Print();
            break;
        }
        case 5: {
            //Query Control
            std::cout << "\033[4;31mControl Query:\033[0m" << std::endl;
            JAUS::QueryControl controlQuery(JAUS::Address(5000,1,1), component.GetComponentID());
            JAUS::ReportControl controlResponse;
            std::cout << "Return: " << component.Send(&controlQuery, &controlResponse) << std::endl;
            controlResponse.Print();
            break;
        }
        case 6: {
            //Query Events
            std::cout << "NOPE LOL" << std::endl;
            break;
        }
        case 7: {
            //Query Local Pose
            std::cout << "\033[4;31mLocal Pose Query:\033[0m" << std::endl;

            //Lets assume we have to recceive the response in a dedicated function

            //Let us assume for a second, that the reason this doesn't work is judge side
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
            std::cout << "Return: " << component.Send(&local_pose_query, &local_pose_response) << std::endl;
            local_pose_response.PrintMessageBody();

            //SO Maybe, MAYBE that didn't work
            //OK it failed miserably
            //So now maybe as an event?
            JAUS::CreateEvent event(JAUS::Address(5000,1,1), component.GetComponentID());
            event.SetQueryMessage(&local_pose_query);
            event.SetRequestID(1);
            std::cout << "Return: " << component.Send(&event, &local_pose_response) << std::endl;
            local_pose_response.Print();
            break;
        }
        case 9: {
            //Time to find out if the auth be working
            std::cout << "\033[4;31mControl Attempt:\033[0m" << std::endl;
            std::cout << "Return: " << accessControl->RequestComponentControl(JAUS::Address(5000,1,1)) << std::endl;
            break;
        }
        case 10: {
            //Set local Pose
            std::cout << "\033[4;31mSetting Local Pose:\033[0m" << std::endl;
            JAUS::SetLocalPose local_pose_set(JAUS::Address(5000, 1, 1), component.GetComponentID());
            JAUS::ReportLocalPose local_pose_response;

            //First we reset, so that the robot's refference point is wherever the hell it is
            JAUS::Point3D resetPosition(0,0,0);
            JAUS::Point3D resetOrientation(0,0,0); //Could elminate what is essentially a duplicate, but readability is nice

            local_pose_set.SetPose(resetPosition, resetOrientation); // 3rd param may be needed. IDK.

            //Two potential problems:
            //1. Does setting the pose actually make JAUS reply with the current pose?
            std::cout << "Return: " << component.Send(&local_pose_set, &local_pose_response) << std::endl;


            JAUS::Point3D additionToLocation(10,10,0); //X Y Z . How much is 10 X? Fuck if I know
            JAUS::Point3D additionToOrientation(0,0,10); //Roll Pitch Yaw. Also this robot identifies as a bottle of deoderant.

            local_pose_set.AddToPose(additionToLocation, additionToOrientation); //3rd param is time. unsure if needed.
            break;
        }
	case 11: {
		JAUS::QueryEvents query(JAUS::Address(5000, 1, 1), component.GetComponentID());
		query.SetQueryType(JAUS::QueryEvents::AllEvents);
		JAUS::ReportEvents report;
		std::cout << "Result: " << component.Send(&query,&report) << std::endl;
		report.PrintMessageBody();
		break;
	}
	case 12: {
		JAUS::CreateEvent create(JAUS::Address(103,1,1), component.GetComponentID());
		create.SetType(JAUS::Events::Type::Periodic);
		create.SetRequestedPeriodicRate(1);
		JAUS::QueryLocalPose query(JAUS::Address(103,1,1), component.GetComponentID());
		create.SetQueryMessage(&query);
		JAUS::ConfirmEventRequest confirm;
		std::cout << "Event create: " << component.Send(&create,&confirm) << std::endl;
		confirm.PrintMessageBody();
		
		JAUS::UpdateEvent update(JAUS::Address(103,1,1), component.GetComponentID());
		update.SetEventID(0);
		update.SetType(JAUS::Events::Type::Periodic);
		update.SetRequestedPeriodicRate(0.5);
		std::cout << "Event update: " << component.Send(&update,&confirm) << std::endl;
		confirm.PrintMessageBody();
	}
        case 100: {
            //Attempt Shutdown of Server
            accessControl->Shutdown();
            break;
        }
        case 101: {
            //Set Emergency
            std::cout << "\033[4;31mSetting Emergency:\033[0m" << std::endl;
            JAUS::SetEmergency setEmergency(JAUS::Address(5000,1,1), component.GetComponentID());
            JAUS::ReportStatus reportStatus;
            std::cout << "Return: " << component.Send(&setEmergency, &reportStatus) << std::endl;
            reportStatus.Print();
            break;
        }
        case 102: {
            //Clear Emergency
            std::cout << "\033[4;31mClearing Emergency:\033[0m" << std::endl;
            JAUS::ClearEmergency clearEmergency(JAUS::Address(5000,1,1), component.GetComponentID());
            JAUS::ReportStatus reportStatus;

            std::cout << "Return: " << component.Send(&clearEmergency, &reportStatus) << std::endl;
            reportStatus.Print();
            break;
        }
        default: {
            std::cout << "\"" << choice << "\" is NOT an option" << std::endl;
            break;
        }
        }
    }
    return 0;
}


