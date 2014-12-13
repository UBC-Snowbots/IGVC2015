#include <iostream>

#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/core/Base.h"
#include <math.h>

#include "JAUSComponent.hpp"

#include <termios.h>

bool processServiceQuery(openjaus::core::QueryServices& query){
	std::cout << "received services query: " << query.toString() << std::endl;
	return false;
}

int main(void)
{
	// Start of user code for main:
	openjaus::system::Application::setTerminalMode();

	try
	{
		openjaus::core::Base component;
		component.setName("Discovery");
		//most peculiar... how do we get the services to be sent as a response? TODO
		std::cout << "Service count: " << component.getServices().size() << " " << component.getImplements()->size() << std::endl;
		std::cout << "Service list: ";
		for(auto& item:component.getServices()){
			std::cout << item.first << ", ";
		}
		std::cout << std::endl;
		component.addMessageCallback(processServiceQuery);
		component.run();

		unsigned char choice = 0;
		while(choice != 27) // ESC
		{
			choice = openjaus::system::Application::getChar();
			switch(choice)
			{
				case 't':
					std::cout << component.getSystemTree()->toString() << std::endl;
					break;

				case 'm':
					std::cout << openjaus::transport::AddressMap::instance().toString();
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
	// End of user code
}
