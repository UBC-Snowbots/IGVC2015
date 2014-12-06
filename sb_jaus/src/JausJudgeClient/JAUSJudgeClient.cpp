#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/mobility/GlobalPoseSensor.h"

using namespace openjaus;

bool componentIdentificationResponse=false;
core::Base component;

bool processIdentificationResponse(openjaus::core::ReportIdentification& report){
	std::cout << "Identification received: " << report.getIdentification() << std::endl;
	std::cout << "Reponse code: " << report.getType() << std::endl;
	componentIdentificationResponse=true;
	return component.storeIdentification(&report);
}

void printMenu()
{
	std::cout << "Menu:" << std::endl;
	std::cout << "t - Print System Tree" << std::endl;
}

int main(void)
{
	system::Application::setTerminalMode();
	
	transport::Address target(155,1,1);

	component.setName("JAUSJudgeClientSim");
	component.addMessageCallback(processIdentificationResponse);
	component.run();
	
	while(!componentIdentificationResponse){
		std::cout << "Polling identification..." << std::endl;
		openjaus::core::QueryIdentification* idQuery = new openjaus::core::QueryIdentification();
		idQuery->setDestination(target);
		component.sendMessage(idQuery);
		system::Time::sleep(5000);
	}
	
	printMenu();

	unsigned char choice = 0;
	while(choice != 27) // ESC
	{
		choice = system::Application::getChar();
		switch(choice)
		{
			case 't':
				std::cout << component.getSystemTree()->toString();
				break;
		}

	}

	return 0;
	// End of user code
}
