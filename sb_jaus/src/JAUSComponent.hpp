#include <openjaus.h>
#include <openjaus/mobility.h>

using namespace std;

class JAUSComponent: public openjaus::core::Base
{
private:
	openjaus::system::Timer *pingTimer;
	short pingId;
public:
	JAUSComponent():
		pingId(0),
		pingTimer(nullptr)
		{
		name = "SnowBots JAUS Component";
		
		auto discoveryService = new openjaus::model::Service();
		discoveryService->setName("Discovery");
		discoveryService->setUri("urn:SnowBots:Discovery");
		discoveryService->setVersionMajor(1);
		discoveryService->setVersionMinor(0);
		implements->push_back(discoveryService);
		
		receive.addMessageCallback(&JAUSComponent::processEcho, this);
		
		addSelfToSystemTree();
	}
	bool processEcho(openjaus::core::ReportHeartbeatPulse &echo){
		std::cout << "Response from: " << echo.getSource().toString() << ": seq=" << echo.getSequenceNumber() << std::endl;
		return true;
	}
	
	void run(){
		Base::run();
		pingTimer = new openjaus::system::Timer(TIMER_METHOD(JAUSComponent, discover), this);
		pingTimer->setInterval(1000);
	}
	void discover(openjaus::system::Timer *timer){
		openjaus::core::QueryHeartbeatPulse *pingMsg = new openjaus::core::QueryHeartbeatPulse();
		pingMsg->setSequenceNumber(pingId);
		broadcastToNode(pingMsg);
		//sendTimeSec.push_back(openjaus::system::Time::getTime().inSec());
		pingId++;
	}
};
