#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/mobility/GlobalPoseSensor.h"

using namespace openjaus;

bool componentIdentificationResponse=false;
core::Base component;

transport::Address target(155,1,1);

bool processIdentificationResponse(openjaus::core::ReportIdentification& report){
	std::cout << "Identification received: " << report.getIdentification() << std::endl;
	std::cout << "Reponse code: " << report.getType() << std::endl;
	componentIdentificationResponse=true;
	bool tmp = component.storeIdentification(&report);
	
	std::cout << "Querying capabilities..." << std::endl;
	auto serviceQuery = new openjaus::core::QueryServices;
	openjaus::core::QSNodeRecord nodeID;
	nodeID.setNodeID(1);
	{
		openjaus::model::fields::UnsignedByte tmp;
		tmp.setValue(1);
		nodeID.getQSComponentList().add(tmp);
	}
	serviceQuery->getQSNodeList().add(nodeID);
	serviceQuery->setDestination(target);
	component.enqueue(serviceQuery);
	
	return tmp;
}

bool processServiceQueryResponse(openjaus::core::ReportServices& report){
	std::cout << "Services received: " << report.toString() << std::endl;
	for(auto& record:report.getRSNodeList().getServicesNodeRec()){
		std::cout << "Node " << (unsigned int)record.getNodeID() << ": " << std::endl;
		for(auto& comp:record.getServicesComponentList().getServicesComponentRec()){
			for(auto& srv:comp.getServicesServiceList().getServicesServiceRec()){
				std::cout << '\t' << comp.getComponentID() << ". URI: " << srv.getUri() << std::endl;
			}
		}
	}
	return true;
}

void printMenu()
{
	std::cout << "Menu:" << std::endl;
	std::cout << "t - Print System Tree" << std::endl;
}

int main(void)
{
	system::Application::setTerminalMode();

	component.setName("JAUSJudgeClientSim");
	component.addMessageCallback(processIdentificationResponse);
	component.addMessageCallback(processServiceQueryResponse);
	component.run();
	
	while(!componentIdentificationResponse){
		std::cout << "Polling identification..." << std::endl;
		openjaus::core::QueryIdentification* idQuery = new openjaus::core::QueryIdentification();
		idQuery->setDestination(target);
		//component.sendMessage(idQuery);
		component.enqueue(idQuery);
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
