/**
\file AccessControlled.cpp

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


#include "openjaus/core/AccessControlled.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace core
{

AccessControlled::AccessControlled() : Events(),
	notControlledLoopback(),
	acceptControlTransition(),
	controlledLoopback(),
	releaseControlTransition(),
	defaultStateLoop(),
	notControlled(),
	controlled(),
	accessStateMachine()
{
	// Add Service Identification Data to implements list
	name = "AccessControlled";
	
	model::Service *accessControlService = new model::Service();
	accessControlService->setName("AccessControl");
	accessControlService->setUri("urn:jaus:jss:core:AccessControl");
	accessControlService->setVersionMajor(1);
	accessControlService->setVersionMinor(1);
	this->implements->push_back(accessControlService);
	
	notControlled.setName("NotControlled");
	controlled.setName("Controlled");
	

	notControlledLoopback.setInterface(this);
	notControlledLoopback.setTransportInterface(this);
	notControlledLoopback.setEndState(&notControlled);
	notControlledLoopback.setStartState(&notControlled);
	notControlled.addTransition(notControlledLoopback);
	
	acceptControlTransition.setInterface(this);
	acceptControlTransition.setTransportInterface(this);
	acceptControlTransition.setEndState(&controlled);
	acceptControlTransition.setStartState(&notControlled);
	notControlled.addTransition(acceptControlTransition);
	
	controlledLoopback.setInterface(this);
	controlledLoopback.setTransportInterface(this);
	controlledLoopback.setEndState(&controlled);
	controlledLoopback.setStartState(&controlled);
	controlled.addTransition(controlledLoopback);
	
	releaseControlTransition.setInterface(this);
	releaseControlTransition.setTransportInterface(this);
	releaseControlTransition.setEndState(&notControlled);
	releaseControlTransition.setStartState(&controlled);
	controlled.addTransition(releaseControlTransition);
	
	defaultStateLoop.setInterface(this);
	defaultStateLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(defaultStateLoop);
	
	accessStateMachine.setName("AccessStateMachine");
	accessStateMachine.addState(notControlled);
	accessStateMachine.addState(controlled);
	accessStateMachine.setStartingState(&notControlled);
	receivingState.addStateMachine(accessStateMachine);
	
    
	notControlled.addEntryAction(&AccessControlled::init, this);
	controlled.addExitAction(&AccessControlled::sendRejectControlReleased, this);
    
	// Start of user code for Constructor:
	// End of user code
}

AccessControlled::~AccessControlled()
{
	// Start of user code for Destructor:
	// End of user code
}

void AccessControlled::sendRejectControlReleased()
{
	// Start of user code for action sendRejectControlReleased():
	// End of user code
}

void AccessControlled::init()
{
	// Start of user code for action init():
	// End of user code
}

core::ReportControl AccessControlled::getReportControl(QueryControl *queryControl)
{
	// Start of user code for action getReportControl(QueryControl *queryControl):
	core::ReportControl message;
	return message;
	// End of user code
}

core::ReportAuthority AccessControlled::getReportAuthority(QueryAuthority *queryAuthority)
{
	// Start of user code for action getReportAuthority(QueryAuthority *queryAuthority):
	core::ReportAuthority message;
	return message;
	// End of user code
}

/// Send action for ReportTimeout with input message SendReportTimeout.
/// \param[in]  sendReportTimeout                 - <describe this>
/// \return     core::ReportTimeout - <describe this>
core::ReportTimeout AccessControlled::getReportTimeout(model::Trigger *trigger)
{
	// Start of user code for action getReportTimeout(model::Trigger *trigger):
	core::ReportTimeout message;
	return message;
	// End of user code
}

bool AccessControlled::setAuthority(RequestControl *requestControl)
{
	// Start of user code for action setAuthority(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::setAuthority(SetAuthority *setAuthority)
{
	// Start of user code for action setAuthority(SetAuthority *setAuthority):
	return false;
	// End of user code
}

bool AccessControlled::resetTimer(RequestControl *requestControl)
{
	// Start of user code for action resetTimer(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::sendConfirmControlNotAvailable(RequestControl *requestControl)
{
	// Start of user code for action sendConfirmControlNotAvailable(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::sendConfirmControlInsufficientAuthority(RequestControl *requestControl)
{
	// Start of user code for action sendConfirmControlInsufficientAuthority(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::sendConfirmControlAccepted(RequestControl *requestControl)
{
	// Start of user code for action sendConfirmControlAccepted(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::sendRejectControlToController(RequestControl *requestControl)
{
	// Start of user code for action sendRejectControlToController(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::sendRejectControlNotAvailable(ReleaseControl *releaseControl)
{
	// Start of user code for action sendRejectControlNotAvailable(ReleaseControl *releaseControl):
	return false;
	// End of user code
}

bool AccessControlled::sendRejectControlReleased(ReleaseControl *releaseControl)
{
	// Start of user code for action sendRejectControlReleased(ReleaseControl *releaseControl):
	return false;
	// End of user code
}

bool AccessControlled::storeAddress(RequestControl *requestControl)
{
	// Start of user code for action storeAddress(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::updateControlledList(ConfirmControl *confirmControl)
{
	// Start of user code for action updateControlledList(ConfirmControl *confirmControl):
	return false;
	// End of user code
}

bool AccessControlled::updateControlledList(RejectControl *rejectControl)
{
	// Start of user code for action updateControlledList(RejectControl *rejectControl):
	return false;
	// End of user code
}


bool AccessControlled::isDefaultAuthorityGreater(RequestControl *requestControl)
{
	// Start of user code for action isDefaultAuthorityGreater(RequestControl *requestControl):
	return false;
	// End of user code
}


bool AccessControlled::isCurrentAuthorityLess(RequestControl *requestControl)
{
	// Start of user code for action isCurrentAuthorityLess(RequestControl *requestControl):
	return false;
	// End of user code
}


bool AccessControlled::isAuthorityValid(SetAuthority *setAuthority)
{
	// Start of user code for action isAuthorityValid(SetAuthority *setAuthority):
	return false;
	// End of user code
}


bool AccessControlled::isControllingClient(RequestControl *requestControl)
{
	// Start of user code for action isControllingClient(RequestControl *requestControl):
	return false;
	// End of user code
}

bool AccessControlled::isControllingClient(SetAuthority *setAuthority)
{
	// Start of user code for action isControllingClient(SetAuthority *setAuthority):
	return false;
	// End of user code
}

bool AccessControlled::isControllingClient(ReleaseControl *releaseControl)
{
	// Start of user code for action isControllingClient(ReleaseControl *releaseControl):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

