#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>

#include "JAUSComponent.hpp"

int main()
{	
	try
	{
		JAUSComponent component;
		component.run();
		std::cout << "Hello world!" << std::endl;
		component.stop();
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}
	
	return 0;
}
