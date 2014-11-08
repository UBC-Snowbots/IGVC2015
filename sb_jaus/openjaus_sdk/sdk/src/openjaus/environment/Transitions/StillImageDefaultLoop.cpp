/**
\file StillImageDefaultLoop.h

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
#include "openjaus/environment/Transitions/StillImageDefaultLoop.h"
#include "openjaus/environment/Triggers/QueryStillImageSensorCapabilities.h"
#include "openjaus/environment/Triggers/QueryStillImageSensorConfiguration.h"
	
namespace openjaus
{
namespace environment
{

StillImageDefaultLoop::StillImageDefaultLoop()
{
	setName("StillImageDefaultLoop");
	setType(model::LOOPBACK);
}

StillImageDefaultLoop::~StillImageDefaultLoop()
{
}

bool StillImageDefaultLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryStillImageSensorCapabilities::ID:
		{
			QueryStillImageSensorCapabilities queryStillImageSensorCapabilities(message);
			
			ReportStillImageSensorCapabilities *reportStillImageSensorCapabilities = new ReportStillImageSensorCapabilities();
			*reportStillImageSensorCapabilities = cmptInterface->getReportStillImageSensorCapabilities(&queryStillImageSensorCapabilities);
			reportStillImageSensorCapabilities->setDestination(message->getSource());
			transport->sendMessage(reportStillImageSensorCapabilities);			
			return true;
			break;
		}
			
		case QueryStillImageSensorConfiguration::ID:
		{
			QueryStillImageSensorConfiguration queryStillImageSensorConfiguration(message);
			
			ReportStillImageSensorConfiguration *reportStillImageSensorConfiguration = new ReportStillImageSensorConfiguration();
			*reportStillImageSensorConfiguration = cmptInterface->getReportStillImageSensorConfiguration(&queryStillImageSensorConfiguration);
			reportStillImageSensorConfiguration->setDestination(message->getSource());
			transport->sendMessage(reportStillImageSensorConfiguration);			
			return true;
			break;
		}
			
		case QueryStillImageData::ID:
		{
			QueryStillImageData queryStillImageData(message);
			
			if(cmptInterface->isCoordinateTransformSupported(&queryStillImageData))
			{
				ReportStillImageData *reportStillImageData = new ReportStillImageData();
				*reportStillImageData = cmptInterface->getReportStillImageData(&queryStillImageData);
				reportStillImageData->setDestination(message->getSource());
				transport->sendMessage(reportStillImageData);
				return true;
			}
			
			if(!cmptInterface->isCoordinateTransformSupported(&queryStillImageData))
			{
				ReportStillImageData *reportStillImageData = new ReportStillImageData();
				*reportStillImageData = cmptInterface->getReportStillImageData(&queryStillImageData);
				reportStillImageData->setDestination(message->getSource());
				transport->sendMessage(reportStillImageData);
				return true;
			}
			break;
		}
			
	}
	
	return false;
}

model::Message* StillImageDefaultLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryStillImageSensorCapabilities::ID:
		{	
			QueryStillImageSensorCapabilities queryStillImageSensorCapabilities(message);
			ReportStillImageSensorCapabilities *reportStillImageSensorCapabilities = new ReportStillImageSensorCapabilities();
			*reportStillImageSensorCapabilities = cmptInterface->getReportStillImageSensorCapabilities(&queryStillImageSensorCapabilities);
			return reportStillImageSensorCapabilities;
		}
			
		case QueryStillImageSensorConfiguration::ID:
		{	
			QueryStillImageSensorConfiguration queryStillImageSensorConfiguration(message);
			ReportStillImageSensorConfiguration *reportStillImageSensorConfiguration = new ReportStillImageSensorConfiguration();
			*reportStillImageSensorConfiguration = cmptInterface->getReportStillImageSensorConfiguration(&queryStillImageSensorConfiguration);
			return reportStillImageSensorConfiguration;
		}
			
		case QueryStillImageData::ID:
		{	
			QueryStillImageData queryStillImageData(message);
			ReportStillImageData *reportStillImageData = new ReportStillImageData();
			*reportStillImageData = cmptInterface->getReportStillImageData(&queryStillImageData);
			return reportStillImageData;
		}
			
	}
	
	return NULL;
}


bool StillImageDefaultLoop::setInterface(StillImageInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool StillImageDefaultLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace environment
} // namespace openjaus

