// File Header Here

#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/core/Base.h"
#include <math.h>

class PingComponent : public openjaus::core::Base
{
public:
	openjaus::system::Timer *pingTimer;
	short pingId;
	std::vector<double> sendTimeSec;

	PingComponent() :
		pingTimer(NULL),
		pingId(0),
		sendTimeSec()
	{
		name = "OpenJAUS Ping Component";

		openjaus::model::Service *pingService = new openjaus::model::Service();
		pingService->setName("Ping");
		pingService->setUri("urn:openjaus:Ping");
		pingService->setVersionMajor(1);
		pingService->setVersionMinor(0);
		implements->push_back(pingService);

		receive.addMessageCallback(&PingComponent::processEcho, this);
		double measureTime = fabs(openjaus::system::Time::getTime().inSec() - openjaus::system::Time::getTime().inSec());
		std::cout << "Measure time=" << 1000.0 * measureTime << " ms" << std::endl;

		double allocTime = -openjaus::system::Time::getTime().inSec();
		openjaus::core::QueryHeartbeatPulse *pingMsg = new openjaus::core::QueryHeartbeatPulse();
		allocTime += openjaus::system::Time::getTime().inSec();
		std::cout << "Alloc time=" << 1000.0 * allocTime << " ms" << std::endl;

		delete pingMsg;
	}

	virtual ~PingComponent()
	{
		if(pingTimer)
		{
			delete pingTimer;
		}
	}

	void run()
	{
		Base::run();
		pingTimer = new openjaus::system::Timer(TIMER_METHOD(PingComponent, ping), this);
		pingTimer->setInterval(1000);
	}

	bool processEcho(openjaus::core::ReportHeartbeatPulse &echo)
	{
		double pingTime = openjaus::system::Time::getTime().inSec() - sendTimeSec[echo.getSequenceNumber()];
		std::cout << "Response from: " << echo.getSource().toString() << ": seq=" << echo.getSequenceNumber() << " time=" << 1000.0 * pingTime << " ms" << std::endl;
		return true;
	}

	void ping(openjaus::system::Timer *timer)
	{
		openjaus::core::QueryHeartbeatPulse *pingMsg = new openjaus::core::QueryHeartbeatPulse();
		pingMsg->setSequenceNumber(pingId);
		broadcastToNode(pingMsg);
		sendTimeSec.push_back(openjaus::system::Time::getTime().inSec());
		pingId++;
	}
};

int main(void)
{
	openjaus::system::Application::setTerminalMode();
	try
	{
		PingComponent component;
		component.run();

		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					LOG(component.getSystemTree()->toString());
					break;
			}
		}
	}
	catch(openjaus::system::Exception& expn)
	{
		openjaus::system::Logger::log(expn);
	}

	return 0;
	// End of user code
}
