#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/core/Base.h"
#include "openjaus/mobility.h"
#include "openjaus/mobility/GlobalPoseSensor.h"

#if defined(WIN32)

#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif

using namespace openjaus;

class GposComponent : public openjaus::mobility::GlobalPoseSensor
{
public:
	mobility::ReportGlobalPose reportGlobalPose;
	mobility::ReportGeomagneticProperty reportGeomagneticProperty;

	GposComponent() :
		GlobalPoseSensor(),
		reportGlobalPose(),
		reportGeomagneticProperty()
	{
		name = "GposDemo";
		this->reportGeomagneticProperty.setMagneticVariation_rad(1.0);

		this->reportGlobalPose.setLatitude_deg(34.5);
		this->reportGlobalPose.setLongitude_deg(-80.04);
		this->reportGlobalPose.setAltitude_m(100);
		this->reportGlobalPose.setRoll_rad(0.5);
		this->reportGlobalPose.setPitch_rad(-0.1);
		this->reportGlobalPose.setYaw_rad(0.1);

		publish(&reportGlobalPose);
	}

    mobility::ReportGlobalPose getReportGlobalPose(mobility::QueryGlobalPose *queryGlobalPose)
    {
    	LOG("Processing Query Global Pose");
    	reportGlobalPose.setPresenceVector(queryGlobalPose->getQueryPresenceVector());
    	return reportGlobalPose;
    }

	mobility::ReportGeomagneticProperty getReportGeomagneticProperty(mobility::QueryGeomagneticProperty *queryGeomagneticProperty)
	{
    	return reportGeomagneticProperty;
	}

	bool updateGlobalPose(mobility::SetGlobalPose *setGlobalPose)
	{
		if(setGlobalPose->isLatitudeEnabled())
		{
			this->reportGlobalPose.setLatitude_deg(setGlobalPose->getLatitude_deg());
		}

		if(setGlobalPose->isLongitudeEnabled())
		{
			this->reportGlobalPose.setLongitude_deg(setGlobalPose->getLongitude_deg());
		}

		if(setGlobalPose->isAltitudeEnabled())
		{
			this->reportGlobalPose.setAltitude_m(setGlobalPose->getAltitude_m());
		}

		if(setGlobalPose->isRollEnabled())
		{
			this->reportGlobalPose.setRoll_rad(setGlobalPose->getRoll_rad());
		}

		if(setGlobalPose->isPitchEnabled())
		{
			this->reportGlobalPose.setPitch_rad(setGlobalPose->getPitch_rad());
		}

		if(setGlobalPose->isYawEnabled())
		{
			this->reportGlobalPose.setYaw_rad(setGlobalPose->getYaw_rad());
		}
		return true;
	}

	bool updateGeomagneticProperty(mobility::SetGeomagneticProperty *setGeomagneticProperty)
	{
		this->reportGeomagneticProperty.setMagneticVariation_rad(setGeomagneticProperty->getMagneticVariation_rad());
		return true;
	}

	bool isControllingGposClient(mobility::SetGlobalPose *setGlobalPose)
	{
		return (setGlobalPose->getSource() == this->controllerAddress);
	}

	bool isControllingGposClient(mobility::SetGeomagneticProperty *setGeomagneticProperty)
	{
		return (setGeomagneticProperty->getSource() == this->controllerAddress);
	}

	void run()
	{
		GlobalPoseSensor::run();
	}
};

int main(void)
{
	openjaus::system::Application::setTerminalMode();

	try
	{
		GposComponent* component = new GposComponent();
		component->run();

		std::cout << "Menu:\n";
		std::cout << "t - Print System Tree\n";
		std::cout << "ESC - Exit Component\n";

		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					std::cout << component->getSystemTree()->toString();
					break;

				case 'm':
					std::cout << openjaus::transport::AddressMap::instance().toString();
					break;
			}
		}

		delete component;
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}

	return 0;
	// End of user code
}
