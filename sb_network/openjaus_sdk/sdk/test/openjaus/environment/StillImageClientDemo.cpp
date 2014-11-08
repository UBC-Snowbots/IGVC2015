#include <openjaus.h>
#include "openjaus/core/Base.h"
#include "openjaus/environment/StillImageSensor.h"
#include <fstream>
#include <iomanip>

using namespace openjaus;

bool processReportStillImageData(environment::ReportStillImageData &report)
{
	std::cout << "Recv Report Still Image Data\n";
	std::cout << report.toXml();

	openjaus::environment::StillImageDataRecord imageDataRecord = report.getStillImageDataList().get(0);
	openjaus::environment::ImageFrameBlob& imageFrame = imageDataRecord.getImageFrame();

	std::ofstream imageFile;
	imageFile.open("output.jpg", std::ios::out | std::ios::binary);
	imageFile.write((const char *)imageFrame.getPayload().getBuffer(), imageFrame.getPayload().containedBytes());
	imageFile.close();
	std::cout << "Write File: output.jpg\n";

	return true;
}

void printMenu()
{
	std::cout << "Menu:" << std::endl;
	std::cout << "t - Print System Tree" << std::endl;
	std::cout << "1 - Query Still Image" << std::endl;
	std::cout << "? - Output Menu" << std::endl;
	std::cout << "ESC - Exit Component" << std::endl;
}

int main(void)
{
	system::Application::setTerminalMode();

	std::vector<transport::Address> serviceList;

	core::Base component;
	component.setName("StillImageClientDemo");
	component.addMessageCallback(processReportStillImageData);
	component.run();

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

			case '?':
				printMenu();
				break;

			case '1':
			{
				serviceList = component.getSystemTree()->lookupService("urn:jaus:jss:environmentSensing:StillImage");
				if(serviceList.size() > 0)
				{
					environment::QueryStillImageData *query = new environment::QueryStillImageData();
					query->setDestination(serviceList.at(0));
					component.sendMessage(query);
					std::cout << "Send Query Still Image Data to " << serviceList.at(0) << std::endl;
				}
				break;
			}

		}

	}

	return 0;
	// End of user code
}
