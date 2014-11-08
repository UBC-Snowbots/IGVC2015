/**
\file GwldDefaultLoop.h

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
#include "openjaus/mobility/Transitions/GwldDefaultLoop.h"
	
namespace openjaus
{
namespace mobility
{

GwldDefaultLoop::GwldDefaultLoop()
{
	setName("GwldDefaultLoop");
	setType(model::LOOPBACK);
}

GwldDefaultLoop::~GwldDefaultLoop()
{
}

bool GwldDefaultLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryTravelSpeed::ID:
		{
			QueryTravelSpeed queryTravelSpeed(message);
			
			ReportTravelSpeed *reportTravelSpeed = new ReportTravelSpeed();
			*reportTravelSpeed = cmptInterface->getReportTravelSpeed(&queryTravelSpeed);
			reportTravelSpeed->setDestination(message->getSource());
			transport->sendMessage(reportTravelSpeed);			
			return true;
			break;
		}
			
		case QueryGlobalWaypoint::ID:
		{
			QueryGlobalWaypoint queryGlobalWaypoint(message);
			
			if(cmptInterface->gwldWaypointExists(&queryGlobalWaypoint))
			{
				ReportGlobalWaypoint *reportGlobalWaypoint = new ReportGlobalWaypoint();
				*reportGlobalWaypoint = cmptInterface->getReportGlobalWaypoint(&queryGlobalWaypoint);
				reportGlobalWaypoint->setDestination(message->getSource());
				transport->sendMessage(reportGlobalWaypoint);
				return true;
			}
			break;
		}
			
		case QueryActiveElement::ID:
		{
			QueryActiveElement queryActiveElement(message);
			
			ReportActiveElement *reportActiveElement = new ReportActiveElement();
			*reportActiveElement = cmptInterface->getReportActiveElement(&queryActiveElement);
			reportActiveElement->setDestination(message->getSource());
			transport->sendMessage(reportActiveElement);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* GwldDefaultLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryTravelSpeed::ID:
		{	
			QueryTravelSpeed queryTravelSpeed(message);
			ReportTravelSpeed *reportTravelSpeed = new ReportTravelSpeed();
			*reportTravelSpeed = cmptInterface->getReportTravelSpeed(&queryTravelSpeed);
			return reportTravelSpeed;
		}
			
		case QueryGlobalWaypoint::ID:
		{	
			QueryGlobalWaypoint queryGlobalWaypoint(message);
			ReportGlobalWaypoint *reportGlobalWaypoint = new ReportGlobalWaypoint();
			*reportGlobalWaypoint = cmptInterface->getReportGlobalWaypoint(&queryGlobalWaypoint);
			return reportGlobalWaypoint;
		}
			
		case QueryActiveElement::ID:
		{	
			QueryActiveElement queryActiveElement(message);
			ReportActiveElement *reportActiveElement = new ReportActiveElement();
			*reportActiveElement = cmptInterface->getReportActiveElement(&queryActiveElement);
			return reportActiveElement;
		}
			
	}
	
	return NULL;
}


bool GwldDefaultLoop::setInterface(GlobalWaypointListDriverInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool GwldDefaultLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace mobility
} // namespace openjaus

