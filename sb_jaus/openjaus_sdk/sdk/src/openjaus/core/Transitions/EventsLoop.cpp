/**
\file EventsLoop.h

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
#include "openjaus/core/Transitions/EventsLoop.h"
#include "openjaus/core/Triggers/CancelEvent.h"
#include "openjaus/core/Triggers/ConfirmEventRequest.h"
#include "openjaus/core/Triggers/CreateEvent.h"
#include "openjaus/core/Triggers/Event.h"
#include "openjaus/core/Triggers/QueryEvents.h"
#include "openjaus/core/Triggers/RejectEventRequest.h"
#include "openjaus/core/Triggers/ReportEvents.h"
#include "openjaus/core/Triggers/UpdateEvent.h"
	
namespace openjaus
{
namespace core
{

EventsLoop::EventsLoop()
{
	setName("EventsLoop");
	setType(model::LOOPBACK);
}

EventsLoop::~EventsLoop()
{
}

bool EventsLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryEvents::ID:
		{
			QueryEvents queryEvents(message);
			
			ReportEvents *reportEvents = new ReportEvents();
			*reportEvents = cmptInterface->getReportEvents(&queryEvents);
			reportEvents->setDestination(message->getSource());
			transport->sendMessage(reportEvents);			
			return true;
			break;
		}
			
		case CreateEvent::ID:
		{
			CreateEvent createEvent(message);
			
			if(cmptInterface->isSupported(&createEvent) && !cmptInterface->eventExists(&createEvent))
			{
				cmptInterface->createEvent(&createEvent);
				ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
				*confirmEventRequest = cmptInterface->getConfirmEventRequest(&createEvent);
				confirmEventRequest->setDestination(message->getSource());
				transport->sendMessage(confirmEventRequest);
				cmptInterface->resetEventTimer(&createEvent);
				return true;
			}
			
			if(cmptInterface->isSupported(&createEvent) && cmptInterface->eventExists(&createEvent))
			{
				cmptInterface->updateEvent(&createEvent);
				ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
				*confirmEventRequest = cmptInterface->getConfirmEventRequest(&createEvent);
				confirmEventRequest->setDestination(message->getSource());
				transport->sendMessage(confirmEventRequest);
				cmptInterface->resetEventTimer(&createEvent);
				return true;
			}
			
			if(!cmptInterface->isSupported(&createEvent))
			{
				RejectEventRequest *rejectEventRequest = new RejectEventRequest();
				*rejectEventRequest = cmptInterface->getRejectEventRequest(&createEvent);
				rejectEventRequest->setDestination(message->getSource());
				transport->sendMessage(rejectEventRequest);
				return true;
			}
			break;
		}
			
		case UpdateEvent::ID:
		{
			UpdateEvent updateEvent(message);
			
			if(cmptInterface->isSupported(&updateEvent) && cmptInterface->eventExists(&updateEvent))
			{
				cmptInterface->updateEvent(&updateEvent);
				ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
				*confirmEventRequest = cmptInterface->getConfirmEventRequest(&updateEvent);
				confirmEventRequest->setDestination(message->getSource());
				transport->sendMessage(confirmEventRequest);
				cmptInterface->resetEventTimer(&updateEvent);
				return true;
			}
			
			if(!cmptInterface->eventExists(&updateEvent) || !cmptInterface->isSupported(&updateEvent))
			{
				RejectEventRequest *rejectEventRequest = new RejectEventRequest();
				*rejectEventRequest = cmptInterface->getRejectEventRequest(&updateEvent);
				rejectEventRequest->setDestination(message->getSource());
				transport->sendMessage(rejectEventRequest);
				return true;
			}
			break;
		}
			
		case CancelEvent::ID:
		{
			CancelEvent cancelEvent(message);
			
			if(cmptInterface->eventExists(&cancelEvent))
			{
				cmptInterface->cancelEvent(&cancelEvent);
				ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
				*confirmEventRequest = cmptInterface->getConfirmEventRequest(&cancelEvent);
				confirmEventRequest->setDestination(message->getSource());
				transport->sendMessage(confirmEventRequest);
				cmptInterface->stopEventTimer(&cancelEvent);
				return true;
			}
			
			if(!cmptInterface->eventExists(&cancelEvent))
			{
				RejectEventRequest *rejectEventRequest = new RejectEventRequest();
				*rejectEventRequest = cmptInterface->getRejectEventRequest(&cancelEvent);
				rejectEventRequest->setDestination(message->getSource());
				transport->sendMessage(rejectEventRequest);
				return true;
			}
			break;
		}
			
		case ConfirmEventRequest::ID:
		{
			ConfirmEventRequest confirmEventRequest(message);
			
			cmptInterface->confirmEvent(&confirmEventRequest);			
			return true;
			break;
		}
			
		case Event::ID:
		{
			Event event(message);
			
			cmptInterface->handleIncomingEvent(&event);			
			return true;
			break;
		}
			
		case ReportEvents::ID:
		{
			ReportEvents reportEvents(message);
			
			cmptInterface->handleReportEvents(&reportEvents);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* EventsLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryEvents::ID:
		{	
			QueryEvents queryEvents(message);
			ReportEvents *reportEvents = new ReportEvents();
			*reportEvents = cmptInterface->getReportEvents(&queryEvents);
			return reportEvents;
		}
			
		case CreateEvent::ID:
		{	
			CreateEvent createEvent(message);
			ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
			*confirmEventRequest = cmptInterface->getConfirmEventRequest(&createEvent);
			return confirmEventRequest;
		}
			
		case UpdateEvent::ID:
		{	
			UpdateEvent updateEvent(message);
			ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
			*confirmEventRequest = cmptInterface->getConfirmEventRequest(&updateEvent);
			return confirmEventRequest;
		}
			
		case CancelEvent::ID:
		{	
			CancelEvent cancelEvent(message);
			ConfirmEventRequest *confirmEventRequest = new ConfirmEventRequest();
			*confirmEventRequest = cmptInterface->getConfirmEventRequest(&cancelEvent);
			return confirmEventRequest;
		}
			
		case ConfirmEventRequest::ID:
			break;
			
		case Event::ID:
			break;
			
		case ReportEvents::ID:
			break;
			
	}
	
	return NULL;
}


bool EventsLoop::setInterface(EventsInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool EventsLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace core
} // namespace openjaus

