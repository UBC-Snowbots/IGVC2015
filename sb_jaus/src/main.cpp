#include <iostream>

#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/core/Base.h"
#include <math.h>

#include "JAUSComponent.hpp"

#include <termios.h>

int main(void)
{
	// Start of user code for main:
	openjaus::system::Application::setTerminalMode();
	try
	{
		JAUSComponent component;
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
