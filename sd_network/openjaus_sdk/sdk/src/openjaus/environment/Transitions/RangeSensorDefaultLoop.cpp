/**
\file RangeSensorDefaultLoop.h

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
#include "openjaus/environment/Transitions/RangeSensorDefaultLoop.h"
#include "openjaus/environment/Triggers/QueryRangeSensorCapabilities.h"
#include "openjaus/environment/Triggers/QueryRangeSensorConfiguration.h"
#include "openjaus/environment/Triggers/QuerySensorGeometricProperties.h"
#include "openjaus/environment/Triggers/QueryRangeSensorData.h"
#include "openjaus/environment/Triggers/QueryRangeSensorCompressedData.h"
	
namespace openjaus
{
namespace environment
{

RangeSensorDefaultLoop::RangeSensorDefaultLoop()
{
	setName("RangeSensorDefaultLoop");
	setType(model::LOOPBACK);
}

RangeSensorDefaultLoop::~RangeSensorDefaultLoop()
{
}

bool RangeSensorDefaultLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryRangeSensorCapabilities::ID:
		{
			QueryRangeSensorCapabilities queryRangeSensorCapabilities(message);
			
			ReportRangeSensorCapabilities *reportRangeSensorCapabilities = new ReportRangeSensorCapabilities();
			*reportRangeSensorCapabilities = cmptInterface->getReportRangeSensorCapabilities(&queryRangeSensorCapabilities);
			reportRangeSensorCapabilities->setDestination(message->getSource());
			transport->sendMessage(reportRangeSensorCapabilities);			
			return true;
			break;
		}
			
		case QueryRangeSensorConfiguration::ID:
		{
			QueryRangeSensorConfiguration queryRangeSensorConfiguration(message);
			
			ReportRangeSensorConfiguration *reportRangeSensorConfiguration = new ReportRangeSensorConfiguration();
			*reportRangeSensorConfiguration = cmptInterface->getReportRangeSensorConfiguration(&queryRangeSensorConfiguration);
			reportRangeSensorConfiguration->setDestination(message->getSource());
			transport->sendMessage(reportRangeSensorConfiguration);			
			return true;
			break;
		}
			
		case QuerySensorGeometricProperties::ID:
		{
			QuerySensorGeometricProperties querySensorGeometricProperties(message);
			
			ReportRangeSensorGeometricProperties *reportRangeSensorGeometricProperties = new ReportRangeSensorGeometricProperties();
			*reportRangeSensorGeometricProperties = cmptInterface->getReportRangeSensorGeometricProperties(&querySensorGeometricProperties);
			reportRangeSensorGeometricProperties->setDestination(message->getSource());
			transport->sendMessage(reportRangeSensorGeometricProperties);			
			return true;
			break;
		}
			
		case QueryRangeSensorData::ID:
		{
			QueryRangeSensorData queryRangeSensorData(message);
			
			if(cmptInterface->isCoordinateTransformSupported(&queryRangeSensorData))
			{
				ReportRangeSensorData *reportRangeSensorData = new ReportRangeSensorData();
				*reportRangeSensorData = cmptInterface->getReportRangeSensorData(&queryRangeSensorData);
				reportRangeSensorData->setDestination(message->getSource());
				transport->sendMessage(reportRangeSensorData);
				return true;
			}
			
			if(!cmptInterface->isCoordinateTransformSupported(&queryRangeSensorData))
			{
				ReportRangeSensorData *reportRangeSensorData = new ReportRangeSensorData();
				*reportRangeSensorData = cmptInterface->getReportRangeSensorData(&queryRangeSensorData);
				reportRangeSensorData->setDestination(message->getSource());
				transport->sendMessage(reportRangeSensorData);
				return true;
			}
			break;
		}
			
		case QueryRangeSensorCompressedData::ID:
		{
			QueryRangeSensorCompressedData queryRangeSensorCompressedData(message);
			
			if(cmptInterface->isCoordinateTransformSupported(&queryRangeSensorCompressedData))
			{
				ReportRangeSensorCompressedData *reportRangeSensorCompressedData = new ReportRangeSensorCompressedData();
				*reportRangeSensorCompressedData = cmptInterface->getReportRangeSensorCompressedData(&queryRangeSensorCompressedData);
				reportRangeSensorCompressedData->setDestination(message->getSource());
				transport->sendMessage(reportRangeSensorCompressedData);
				return true;
			}
			
			if(!cmptInterface->isCoordinateTransformSupported(&queryRangeSensorCompressedData))
			{
				ReportRangeSensorCompressedData *reportRangeSensorCompressedData = new ReportRangeSensorCompressedData();
				*reportRangeSensorCompressedData = cmptInterface->getReportRangeSensorCompressedData(&queryRangeSensorCompressedData);
				reportRangeSensorCompressedData->setDestination(message->getSource());
				transport->sendMessage(reportRangeSensorCompressedData);
				return true;
			}
			break;
		}
			
	}
	
	return false;
}

model::Message* RangeSensorDefaultLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryRangeSensorCapabilities::ID:
		{	
			QueryRangeSensorCapabilities queryRangeSensorCapabilities(message);
			ReportRangeSensorCapabilities *reportRangeSensorCapabilities = new ReportRangeSensorCapabilities();
			*reportRangeSensorCapabilities = cmptInterface->getReportRangeSensorCapabilities(&queryRangeSensorCapabilities);
			return reportRangeSensorCapabilities;
		}
			
		case QueryRangeSensorConfiguration::ID:
		{	
			QueryRangeSensorConfiguration queryRangeSensorConfiguration(message);
			ReportRangeSensorConfiguration *reportRangeSensorConfiguration = new ReportRangeSensorConfiguration();
			*reportRangeSensorConfiguration = cmptInterface->getReportRangeSensorConfiguration(&queryRangeSensorConfiguration);
			return reportRangeSensorConfiguration;
		}
			
		case QuerySensorGeometricProperties::ID:
		{	
			QuerySensorGeometricProperties querySensorGeometricProperties(message);
			ReportRangeSensorGeometricProperties *reportRangeSensorGeometricProperties = new ReportRangeSensorGeometricProperties();
			*reportRangeSensorGeometricProperties = cmptInterface->getReportRangeSensorGeometricProperties(&querySensorGeometricProperties);
			return reportRangeSensorGeometricProperties;
		}
			
		case QueryRangeSensorData::ID:
		{	
			QueryRangeSensorData queryRangeSensorData(message);
			ReportRangeSensorData *reportRangeSensorData = new ReportRangeSensorData();
			*reportRangeSensorData = cmptInterface->getReportRangeSensorData(&queryRangeSensorData);
			return reportRangeSensorData;
		}
			
		case QueryRangeSensorCompressedData::ID:
		{	
			QueryRangeSensorCompressedData queryRangeSensorCompressedData(message);
			ReportRangeSensorCompressedData *reportRangeSensorCompressedData = new ReportRangeSensorCompressedData();
			*reportRangeSensorCompressedData = cmptInterface->getReportRangeSensorCompressedData(&queryRangeSensorCompressedData);
			return reportRangeSensorCompressedData;
		}
			
	}
	
	return NULL;
}


bool RangeSensorDefaultLoop::setInterface(RangeSensorInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool RangeSensorDefaultLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace environment
} // namespace openjaus

