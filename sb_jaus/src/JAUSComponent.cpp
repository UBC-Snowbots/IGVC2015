#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>
#include <openjaus/mobility.h>
#include <openjaus/mobility/GlobalPoseSensor.h>

#include "JAUSComponent.hpp"

#if defined(WIN32)

#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif

/*
using namespace openjaus;
using namespace std;

JAUSComponent::JAUSComponent():
	       Base()
{
	name = "SnowBotServer";

	cout << "JAUSComponent created" << std::endl;
	setName("JAUSComponent");
}*/

void JAUSComponent::run()
{
	super::run();
	cout << "JAUS run" << std::endl;	
}


JAUSComponent::~JAUSComponent()	
{

}
