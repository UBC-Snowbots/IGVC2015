#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/mobility/GlobalPoseSensor.h"

using namespace openjaus;

bool componentIdentificationResponse=false;

bool processIdentificationResponse(openjaus::core::ReportIdentification& report){
	std::cout << "Identification received: " << report.getIdentification() << std::endl;
	std::cout << "Reponse code: " << report.getType() << std::endl;
	componentIdentificationResponse=true;
	return true;
}

void printMenu()
{
	std::cout << "Menu:" << std::endl;
	std::cout << "t - Print System Tree" << std::endl;
	std::cout << "1 - Find Global Pose Sensor" << std::endl;
	std::cout << "2 - Query Global Pose" << std::endl;
	std::cout << "3 - Query Geomagnetic Property" << std::endl;
	std::cout << "4 - Create Periodic Event (Report GPOS)" << std::endl;
	std::cout << "5 - Stop Periodic Event (Report GPOS)" << std::endl;
	std::cout << "6 - Request GPOS Control" << std::endl;
	std::cout << "7 - Release GPOS Control" << std::endl;
	std::cout << "8 - Set Global Pose" << std::endl;
	std::cout << "? - Output Menu" << std::endl;
	std::cout << "ESC - Exit Component" << std::endl;
}

int main(void)
{
	system::Application::setTerminalMode();
	
	transport::Address target(1,1,1);

	core::Base component;
	component.setName("JAUSJudgeClientSim");
	component.addMessageCallback(processIdentificationResponse);
	component.run();
	
	while(!componentIdentificationResponse){
		std::cout << "Polling identification..." << std::endl;
		openjaus::core::QueryIdentification* idQuery = new openjaus::core::QueryIdentification();
		//idQuery->setDestination(target);
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
