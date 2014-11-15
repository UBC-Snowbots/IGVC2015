#include <iostream>
#include <openjaus.h>
#include <openjaus/core.h>
#include <openjaus/core/Base.h>

#include "JAUSComponent.hpp"

using namespace std;
int main()
{	
	openjaus::system::Application::setTerminalMode();

	try
	{
		JAUSComponent component;
		component.run();
		std::cout << "Hello world!" << std::endl;
		
		// from openjaus::core::Base
		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					LOG(component.getSystemTree()->toString());
					break;

				case 'm':
					std::cout << openjaus::transport::AddressMap::instance().toString();
					break;

				case 'c':
					component.requestControl(openjaus::transport::Address(1, 1, 1)); // test
					openjaus::transport::Address(1, 1, 1).isValid();
					break;

				case 'r':
					component.releaseControl(openjaus::transport::Address(1, 1, 1)); // test
					break;

				case 'a':
					LOG(openjaus::transport::AddressMap::instance().toString());
					break;

				case 's':
				{
					openjaus::core::QueryHeartbeatPulse *qhb = new openjaus::core::QueryHeartbeatPulse();
					qhb->setDestination(openjaus::transport::Address(1,1,2));
					qhb->setMustArrive(true);
					component.sendMessage(qhb);
					break;
				}
			}
		}
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}
	
	return 0;
}
