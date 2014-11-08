/**
\file AnalogVideoDefaultLoop.h

\par Copyright
Copyright (c) 2012, OpenJAUS, LLC
All rights reserved.

This file is part of the OpenJAUS Software Development Kit (SDK). This 
software is distributed under one of two licenses, the OpenJAUS SDK 
Commercial End User License Agreement or the OpenJAUS SDK Non-Commercial 
End User License Agreement. The appropriate licensing details were included 
in with your developer credentials and software download. See the LICENSE 
file included with this software for full details.
 
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
- [2011-08-23] - Added AS6057: Manipulators
- [2011-08-01] - Added AS6060: Environment Sensing
- [2011-06-16] - First Release 

*/

#include <openjaus.h>
#include "openjaus/environment/Transitions/AnalogVideoDefaultLoop.h"
#include "openjaus/environment/Triggers/QueryAnalogVideoSensorCapabilities.h"
#include "openjaus/environment/Triggers/QueryAnalogVideoSensorConfiguration.h"
	
namespace openjaus
{
namespace environment
{

AnalogVideoDefaultLoop::AnalogVideoDefaultLoop()
{
	setName("AnalogVideoDefaultLoop");
	setType(model::LOOPBACK);
}

AnalogVideoDefaultLoop::~AnalogVideoDefaultLoop()
{
}

bool AnalogVideoDefaultLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryAnalogVideoSensorCapabilities::ID:
		{
			QueryAnalogVideoSensorCapabilities queryAnalogVideoSensorCapabilities(message);
			
			ReportAnalogVideoSensorCapabilities *reportAnalogVideoSensorCapabilities = new ReportAnalogVideoSensorCapabilities();
			*reportAnalogVideoSensorCapabilities = cmptInterface->getReportAnalogVideoSensorCapabilities(&queryAnalogVideoSensorCapabilities);
			reportAnalogVideoSensorCapabilities->setDestination(message->getSource());
			transport->sendMessage(reportAnalogVideoSensorCapabilities);			
			return true;
			break;
		}
			
		case QueryAnalogVideoSensorConfiguration::ID:
		{
			QueryAnalogVideoSensorConfiguration queryAnalogVideoSensorConfiguration(message);
			
			ReportAnalogVideoSensorConfiguration *reportAnalogVideoSensorConfiguration = new ReportAnalogVideoSensorConfiguration();
			*reportAnalogVideoSensorConfiguration = cmptInterface->getReportAnalogVideoSensorConfiguration(&queryAnalogVideoSensorConfiguration);
			reportAnalogVideoSensorConfiguration->setDestination(message->getSource());
			transport->sendMessage(reportAnalogVideoSensorConfiguration);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* AnalogVideoDefaultLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryAnalogVideoSensorCapabilities::ID:
		{	
			QueryAnalogVideoSensorCapabilities queryAnalogVideoSensorCapabilities(message);
			ReportAnalogVideoSensorCapabilities *reportAnalogVideoSensorCapabilities = new ReportAnalogVideoSensorCapabilities();
			*reportAnalogVideoSensorCapabilities = cmptInterface->getReportAnalogVideoSensorCapabilities(&queryAnalogVideoSensorCapabilities);
			return reportAnalogVideoSensorCapabilities;
		}
			
		case QueryAnalogVideoSensorConfiguration::ID:
		{	
			QueryAnalogVideoSensorConfiguration queryAnalogVideoSensorConfiguration(message);
			ReportAnalogVideoSensorConfiguration *reportAnalogVideoSensorConfiguration = new ReportAnalogVideoSensorConfiguration();
			*reportAnalogVideoSensorConfiguration = cmptInterface->getReportAnalogVideoSensorConfiguration(&queryAnalogVideoSensorConfiguration);
			return reportAnalogVideoSensorConfiguration;
		}
			
	}
	
	return NULL;
}


bool AnalogVideoDefaultLoop::setInterface(AnalogVideoInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool AnalogVideoDefaultLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace environment
} // namespace openjaus

