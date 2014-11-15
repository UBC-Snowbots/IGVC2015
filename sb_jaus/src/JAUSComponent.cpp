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

JAUSComponent::JAUSComponent():
	       Base()
{
	cout << "JAUSComponent created" << std::endl;
	setName("JAUSComponent");
}

/*void JAUSComponent::run()
{
	cout << "JAUS run" << std::endl;	
}

void JAUSComponent::stop()
{
	cout << "JAUS stop" << std::endl;
}*/	


/*
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
}*/
