#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>
#include <openjaus/mobility.h>

#include "JAUSComponent.hpp"

#if defined(WIN32)

#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif


using namespace openjaus;
using namespace std;

openjaus::system::Timer *pingTimer;
short pingId;

JAUSComponent::JAUSComponent():
	       Base(),
	       pingTimer(nullptr),		
	       pingId(0)
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

bool JAUSComponent::processEcho(openjaus::core::ReportHeartbeatPulse &echo)
{
	std::cout << "Response from: " << echo.getSource().toString() << ": seq=" << echo.getSequenceNumber() << std::endl;
	return true;
}

void JAUSComponent::run()
{
	Base::run();
	pingTimer = new openjaus::system::Timer(TIMER_METHOD(JAUSComponent, discover), this);
	pingTimer->setInterval(1000);	
}

void JAUSComponent::discover(openjaus::system::Timer *timer)
{
	openjaus::core::QueryHeartbeatPulse *pingMsg = new openjaus::core::QueryHeartbeatPulse();
	pingMsg->setSequenceNumber(pingId);
	broadcastToNode(pingMsg);
	//sendTimeSec.push_back(openjaus::system::Time::getTime().inSec());
	pingId++;
}

JAUSComponent::~JAUSComponent()	
{

}
