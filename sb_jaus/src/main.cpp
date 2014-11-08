#include <iostream>
#include <openjaus.h>
#include <openjaus/core/Base.h>

#include "JAUSComponent.hpp"

int main(){
	JAUSComponent component;
	component.run();
	std::cout << "Hello world!" << std::endl;
	//component.stop(); this doesn't link for some reason
	return 0;
}
