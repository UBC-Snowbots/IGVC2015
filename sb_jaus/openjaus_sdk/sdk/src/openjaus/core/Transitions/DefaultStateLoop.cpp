/**
\file DefaultStateLoop.h

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
#include "openjaus/core/Transitions/DefaultStateLoop.h"
#include "openjaus/core/Triggers/RequestControl.h"
#include "openjaus/core/Triggers/ReleaseControl.h"
#include "openjaus/core/Triggers/QueryControl.h"
	
namespace openjaus
{
namespace core
{

DefaultStateLoop::DefaultStateLoop()
{
	setName("DefaultStateLoop");
	setType(model::LOOPBACK);
}

DefaultStateLoop::~DefaultStateLoop()
{
}

bool DefaultStateLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case RequestControl::ID:
		{
			RequestControl requestControl(message);
			
			cmptInterface->sendConfirmControlNotAvailable(&requestControl);			
			return true;
			break;
		}
			
		case ReleaseControl::ID:
		{
			ReleaseControl releaseControl(message);
			
			cmptInterface->sendRejectControlNotAvailable(&releaseControl);			
			return true;
			break;
		}
			
		case QueryControl::ID:
		{
			QueryControl queryControl(message);
			
			ReportControl *reportControl = new ReportControl();
			*reportControl = cmptInterface->getReportControl(&queryControl);
			reportControl->setDestination(message->getSource());
			transport->sendMessage(reportControl);			
			return true;
			break;
		}
			
		case QueryAuthority::ID:
		{
			QueryAuthority queryAuthority(message);
			
			ReportAuthority *reportAuthority = new ReportAuthority();
			*reportAuthority = cmptInterface->getReportAuthority(&queryAuthority);
			reportAuthority->setDestination(message->getSource());
			transport->sendMessage(reportAuthority);			
			return true;
			break;
		}
			
		case ConfirmControl::ID:
		{
			ConfirmControl confirmControl(message);
			
			cmptInterface->updateControlledList(&confirmControl);			
			return true;
			break;
		}
			
		case RejectControl::ID:
		{
			RejectControl rejectControl(message);
			
			cmptInterface->updateControlledList(&rejectControl);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* DefaultStateLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case RequestControl::ID:
			break;
			
		case ReleaseControl::ID:
			break;
			
		case QueryControl::ID:
		{	
			QueryControl queryControl(message);
			ReportControl *reportControl = new ReportControl();
			*reportControl = cmptInterface->getReportControl(&queryControl);
			return reportControl;
		}
			
		case QueryAuthority::ID:
		{	
			QueryAuthority queryAuthority(message);
			ReportAuthority *reportAuthority = new ReportAuthority();
			*reportAuthority = cmptInterface->getReportAuthority(&queryAuthority);
			return reportAuthority;
		}
			
		case ConfirmControl::ID:
			break;
			
		case RejectControl::ID:
			break;
			
	}
	
	return NULL;
}


bool DefaultStateLoop::setInterface(AccessControlInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool DefaultStateLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace core
} // namespace openjaus

