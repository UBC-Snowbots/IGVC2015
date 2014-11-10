#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/mobility/GlobalPoseSensor.h"

using namespace openjaus;

bool processReportGlobalPose(mobility::ReportGlobalPose &report)
{
	std::cout << "Recv Report Global Pose\n";
	std::cout << "Latitude: " << report.getLatitude_deg() << std::endl;
	std::cout << "Longitude: " << report.getLongitude_deg() << std::endl;
	std::cout << "Altitude: " << report.getAltitude_m() << std::endl;
	std::cout << "Roll: " << report.getRoll_rad() << std::endl;
	std::cout << "Pitch: " << report.getPitch_rad() << std::endl;
	std::cout << "Yaw: " << report.getYaw_rad() << std::endl;
	return true;
}

bool processReportGeomagneticProperty(mobility::ReportGeomagneticProperty &report)
{
	std::cout << "Recv Report Geomagnetic Property\n";
	std::cout << "Magnetic Variation: " << report.getMagneticVariation_rad() << std::endl;
	return true;
}

void processControlResponse(const model::ControlResponse& response)
{
	std::cout << "Recv Control Request Response from: " << response.getAddress() << std::endl;
	std::cout << "Response code: " << response.getResponseType() << std::endl;
}

void printMenu()
{
	std::cout << "Menu:" << std::endl;
	std::cout << "t - Print System Tree" << std::endl;
	std::cout << "1 - Find Global Pose Sensor" << std::endl;
	std::cout << "2 - Query Global Pose" << std::endl;
	std::cout << "3 - Query Geomagnetic Property" << std::endl;
	std::cout << "4 - Create Periodic Event (Report GPOS)" << std::endl;
	std::cout << "5 - Stop Periodic Event (Report GPOS)" << std::endl;
	std::cout << "6 - Request GPOS Control" << std::endl;
	std::cout << "7 - Release GPOS Control" << std::endl;
	std::cout << "8 - Set Global Pose" << std::endl;
	std::cout << "? - Output Menu" << std::endl;
	std::cout << "ESC - Exit Component" << std::endl;
}

/*
int main(void)
{
	system::Application::setTerminalMode();

	std::vector<transport::Address> gposList;

	core::Base component;
	component.setName("JAUSJudgeClientSim");
	component.addMessageCallback(processReportGlobalPose);
	component.addMessageCallback(processReportGeomagneticProperty);
	component.run();
	uint32_t subscriptionId = 0;

	printMenu();

	unsigned char choice = 0;
	while(choice != 27) // ESC
	{
		choice = system::Application::getChar();
		switch(choice)
		{
			case 't':
				std::cout << component.getSystemTree()->toString();
				break;

			case '?':
				printMenu();
				break;

			case '1':
			{
				gposList = component.getSystemTree()->lookupService("urn:jaus:jss:mobility:GlobalPoseSensor");
				std::cout << "GPOS Services (" << gposList.size() << "):\n";
				for(size_t i = 0; i < gposList.size(); i++)
				{
					std::cout << "\t" << gposList.at(i).toString() << std::endl;
				}
				break;
			}

			case '2':
				std::cout << "Sending Query Global Pose" << std::endl;
				if(gposList.size() > 0)
				{
					mobility::QueryGlobalPose *query = new mobility::QueryGlobalPose();
					query->setQueryPresenceVector(65535);
					query->setDestination(gposList.at(0));
					component.sendMessage(query);
				}
				break;

			case '3':
				std::cout << "Sending Query Geomagnetic Property" << std::endl;
				if(gposList.size() > 0)
				{
					mobility::QueryGeomagneticProperty *query = new mobility::QueryGeomagneticProperty();
					query->setDestination(gposList.at(0));
					component.sendMessage(query);
				}
				break;

			case '4':
				if(!subscriptionId && gposList.size() > 0)
				{
					mobility::QueryGlobalPose *query = new mobility::QueryGlobalPose();
					query->setQueryPresenceVector(65535);
					subscriptionId = component.subscribePeriodic(gposList.at(0), query, 1.0);
					std::cout << "Created Periodic Event: " << subscriptionId << std::endl;
				}
				break;

			case '5':
				if(subscriptionId)
				{	std::cout << "Un-subscribing Periodic Event" << std::endl;
					if(component.unsubscribe(subscriptionId))
					{
						subscriptionId = 0;
					}
				}
				break;

			case '6':
				if(gposList.size() > 0)
				{
					component.requestControl(gposList.at(0), processControlResponse);
					std::cout << "Request Control Gpos at " << gposList.at(0) << std::endl;
				}
				break;

			case '7':
				if(gposList.size() > 0)
				{
					try
					{
						component.releaseControl(gposList.at(0));
						std::cout << "Sent Release Control to " << gposList.at(0) << std::endl;
					}
					catch(system::Exception e)
					{
						std::cout << e.toString() << std::endl;
					}
				}
				break;

			case '8':
				if(gposList.size() > 0)
				{
					mobility::SetGlobalPose *pose = new mobility::SetGlobalPose();

					pose->enableLatitude();
					pose->setLatitude_deg(50.0);

					pose->enableLongitude();
					pose->setLongitude_deg(50.0);

					pose->setDestination(gposList.at(0));
					component.sendMessage(pose);
				}
				break;
		}

	}

	return 0;
	// End of user code
}*/
