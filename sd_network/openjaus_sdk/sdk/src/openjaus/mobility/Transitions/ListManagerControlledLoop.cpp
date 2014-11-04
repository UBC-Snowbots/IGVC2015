/**
\file ListManagerControlledLoop.h

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
#include "openjaus/mobility/Transitions/ListManagerControlledLoop.h"
#include "openjaus/mobility/Triggers/SetElement.h"
#include "openjaus/mobility/Triggers/DeleteElement.h"
	
namespace openjaus
{
namespace mobility
{

ListManagerControlledLoop::ListManagerControlledLoop()
{
	setName("ListManagerControlledLoop");
	setType(model::LOOPBACK);
}

ListManagerControlledLoop::~ListManagerControlledLoop()
{
}

bool ListManagerControlledLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case SetElement::ID:
		{
			SetElement setElement(message);
			
			if((cmptInterface->isElementSupported(&setElement) && cmptInterface->isValidElementRequest(&setElement)) && cmptInterface->isControllingListClient(&setElement))
			{
				cmptInterface->setElement(&setElement);
				ConfirmElementRequest *confirmElementRequest = new ConfirmElementRequest();
				*confirmElementRequest = cmptInterface->getConfirmElementRequest(&setElement);
				confirmElementRequest->setDestination(message->getSource());
				transport->sendMessage(confirmElementRequest);
				return true;
			}
			
			if(cmptInterface->isControllingListClient(&setElement) && !cmptInterface->isValidElementRequest(&setElement))
			{
				RejectElementRequest *rejectElementRequest = new RejectElementRequest();
				*rejectElementRequest = cmptInterface->getRejectElementRequest(&setElement);
				rejectElementRequest->setDestination(message->getSource());
				transport->sendMessage(rejectElementRequest);
				return true;
			}
			
			if(!cmptInterface->elementExists(&setElement) && cmptInterface->isControllingListClient(&setElement))
			{
				RejectElementRequest *rejectElementRequest = new RejectElementRequest();
				*rejectElementRequest = cmptInterface->getRejectElementRequest(&setElement);
				rejectElementRequest->setDestination(message->getSource());
				transport->sendMessage(rejectElementRequest);
				return true;
			}
			break;
		}
			
		case DeleteElement::ID:
		{
			DeleteElement deleteElement(message);
			
			if(cmptInterface->elementExists(&deleteElement) && cmptInterface->isControllingListClient(&deleteElement))
			{
				cmptInterface->deleteElement(&deleteElement);
				ConfirmElementRequest *confirmElementRequest = new ConfirmElementRequest();
				*confirmElementRequest = cmptInterface->getConfirmElementRequest(&deleteElement);
				confirmElementRequest->setDestination(message->getSource());
				transport->sendMessage(confirmElementRequest);
				return true;
			}
			
			if(cmptInterface->isControllingListClient(&deleteElement) && !cmptInterface->elementExists(&deleteElement))
			{
				RejectElementRequest *rejectElementRequest = new RejectElementRequest();
				*rejectElementRequest = cmptInterface->getRejectElementRequest(&deleteElement);
				rejectElementRequest->setDestination(message->getSource());
				transport->sendMessage(rejectElementRequest);
				return true;
			}
			break;
		}
			
	}
	
	return false;
}

model::Message* ListManagerControlledLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case SetElement::ID:
		{	
			SetElement setElement(message);
			ConfirmElementRequest *confirmElementRequest = new ConfirmElementRequest();
			*confirmElementRequest = cmptInterface->getConfirmElementRequest(&setElement);
			return confirmElementRequest;
		}
			
		case DeleteElement::ID:
		{	
			DeleteElement deleteElement(message);
			ConfirmElementRequest *confirmElementRequest = new ConfirmElementRequest();
			*confirmElementRequest = cmptInterface->getConfirmElementRequest(&deleteElement);
			return confirmElementRequest;
		}
			
	}
	
	return NULL;
}


bool ListManagerControlledLoop::setInterface(ListManagerInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool ListManagerControlledLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace mobility
} // namespace openjaus

