/**
\file StillImageSensorDemo.cpp

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of OpenJAUS. OpenJAUS is distributed under the OpenJAUS
SDK Commercial End User License Agreement. See the LICENSE.txt file for more 
details.
 
THIS SOFTWARE IS PROVIDED BY THE LICENSOR (OPENJAUS LCC) "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE LICENSOR BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
OUT OF THE USE OF THE SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE. THE LICENSOR DOES NOT WARRANT THAT THE LICENSED SOFTWARE WILL MEET
LICENSEE'S REQUIREMENTS OR THAT THE OPERATION OF THE LICENSED SOFTWARE
WILL BE UNINTERRUPTED OR ERROR-FREE, OR THAT ERRORS IN THE LICENSED
SOFTWARE WILL BE CORRECTED.

\ Software History
- [2012-01-24] - Added to repository to test Large Message Handler

*/

#include <openjaus.h>
#include "openjaus/core.h"
#include "openjaus/environment/StillImageSensor.h"
#include <fstream>
#include <iomanip>

#if defined(WIN32)

#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <termios.h>
#endif

class StillImageComponent : public openjaus::environment::StillImageSensor
{
public:
	openjaus::environment::StillImageDataRecord imageDataRecord;

	StillImageComponent() :
		StillImageSensor()
	{
		name = "StillImageDemo";
		imageDataRecord.setSensorID(1);
		imageDataRecord.setReportCoordinateSystem(openjaus::environment::ReportCoordinateSystemRefEnumeration::NATIVE_COORDINATE_SYSTEM);
		imageDataRecord.disableTimeStamp();
		openjaus::environment::ImageFrameBlob& imageFrame = imageDataRecord.getImageFrame();

		// Image File
		std::ifstream imageFile;

		std::cout << "Open File\n";
		imageFile.open("test.jpg", std::ios::in | std::ios::binary);

		std::cout << "File Opened\n";
		// get length of file
		imageFile.seekg(0, std::ios::end);
		int length = imageFile.tellg();
		imageFile.seekg(0, std::ios::beg);

		std::cout << "File Length: " << length << std::endl;

		// Image Data Buffer
		openjaus::system::Buffer imageData(length);

		// Read binary data
		imageFile.read((char *)imageData.getBuffer(), length);
		imageData.increment(length);
		imageFile.close();

//		std::cout << imageData.toString() << std::endl;
//		for(int i = 0; i < 100; i++)
//		{
//			if(i % 32 == 0)
//			{
//				std::cout << "\n";
//				std::cout << std::setfill('0') << std::setw(6) << std::dec << i << ": ";
//			}
//			else if(i % 8 == 0)
//			{
//				std::cout << " ";
//			}
//			std::cout << std::setw(2) << std::setfill('0') << std::uppercase << std::hex << (uint16_t)(imageData.getBuffer()[i]) << " ";
//		}
//		std::cout << "\n";

		// Set Blob Payload
		imageFrame.setPayload(openjaus::environment::ImageFrameBlob::JPEG, imageData);
		std::cout << "Exit Constructor\n";
	}

	openjaus::environment::ReportStillImageData getReportStillImageData(openjaus::environment::QueryStillImageData *query)
	{
		std::cout << "Handle: getReportStillImageData\n";
		openjaus::environment::ReportStillImageData response;
		openjaus::environment::StillImageDataList &list = response.getStillImageDataList();
		list.add(imageDataRecord);
		std::cout << response.toXml();
		return response;
	}

	openjaus::environment::ReportStillImageSensorCapabilities getReportStillImageSensorCapabilities(openjaus::environment::QueryStillImageSensorCapabilities *query)
	{
		std::cout << "Demo does not handle: QueryStillImageSensorCapabilities\n";
		openjaus::environment::ReportStillImageSensorCapabilities report;
		return report;
	}

	openjaus::environment::ReportStillImageSensorConfiguration getReportStillImageSensorConfiguration(openjaus::environment::QueryStillImageSensorConfiguration *query)
	{
		std::cout << "Demo does not handle: QueryStillImageSensorConfiguration\n";
		openjaus::environment::ReportStillImageSensorConfiguration report;
		return report;
	}

	openjaus::environment::ConfirmStillImageSensorConfiguration getConfirmStillImageSensorConfiguration(openjaus::environment::SetStillImageSensorConfiguration *command)
	{
		std::cout << "Demo does not handle: SetStillImageSensorConfiguration\n";
		openjaus::environment::ConfirmStillImageSensorConfiguration response;
		return response;
	}

	bool updateStillImageSensorConfiguration(openjaus::environment::SetStillImageSensorConfiguration *command)
	{
		std::cout << "Demo does not handle: QueryStillImageSensorCapabilities\n";
		return false;
	}

	openjaus::environment::ReportStillImageData getReportStillImageDataInNativeSystem(openjaus::environment::QueryStillImageData *query)
	{
		std::cout << "Handle: getReportStillImageDataInNativeSystem\n";
		openjaus::environment::ReportStillImageData response;
		openjaus::environment::StillImageDataList &list = response.getStillImageDataList();
		list.add(imageDataRecord);
		std::cout << response.toXml();
		return response;
	}

	openjaus::environment::ReportVisualSensorGeometricProperties getReportVisualSensorGeometricProperties(openjaus::environment::QueryVisualSensorGeometricProperties *query)
	{
		std::cout << "Demo does not handle: QueryVisualSensorGeometricProperties\n";
		openjaus::environment::ReportVisualSensorGeometricProperties response;
		return response;
	}

    openjaus::environment::ConfirmVisualSensorConfiguration getConfirmVisualSensorConfiguration(openjaus::environment::SetVisualSensorConfiguration *command)
    {
		std::cout << "Demo does not handle: SetVisualSensorConfiguration\n";
		openjaus::environment::ConfirmVisualSensorConfiguration response;
		return response;
    }

    openjaus::environment::ReportVisualSensorConfiguration getReportVisualSensorConfiguration(openjaus::environment::QueryVisualSensorConfiguration *query)
    {
		std::cout << "Demo does not handle: QueryVisualSensorConfiguration\n";
		openjaus::environment::ReportVisualSensorConfiguration response;
		return response;
    }

    bool updateVisualSensorConfiguration(openjaus::environment::SetVisualSensorConfiguration *command)
    {
		std::cout << "Demo does not handle: SetVisualSensorConfiguration\n";
		return false;
    }

    openjaus::environment::ReportVisualSensorCapabilities getReportVisualSensorCapabilities(openjaus::environment::QueryVisualSensorCapabilities *query)
    {
		std::cout << "Demo does not handle: QueryVisualSensorCapabilities\n";
		openjaus::environment::ReportVisualSensorCapabilities response;
		return response;
    }

	bool isControllingStillImageClient(openjaus::environment::SetStillImageSensorConfiguration *command)
    {
		return (command->getSource() == this->controllerAddress);
    }

	bool isCoordinateTransformSupported(openjaus::environment::QueryStillImageData *query)
    {
		return false;
    }

	bool isControllingVisualSensorClient(openjaus::environment::SetVisualSensorConfiguration *query)
    {
		return (query->getSource() == this->controllerAddress);
    }

	void run()
	{
		StillImageSensor::run();
	}
};


int main(void)
{
	openjaus::system::Application::setTerminalMode();

	try
	{
		StillImageComponent* component = new StillImageComponent();
		component->run();

		std::cout << "Menu:\n";
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
			}
		}

		delete component;
	}
	catch(openjaus::system::Exception &expn)
	{
		openjaus::system::Logger::log(expn);
	}

	return 0;
	// End of user code
}


