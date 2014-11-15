#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>

#include "JAUSComponent.hpp"

int main()
{	
	try
	{
		openjaus::core::Base component;
		component.run();
		std::cout << "Hello world!" << std::endl;
	}
	catch(openjaus::system::Exception expn)
	{
		openjaus::system::Logger::log(expn);
	}
	
	return 0;
}
