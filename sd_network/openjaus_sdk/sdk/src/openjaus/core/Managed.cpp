/**
\file Managed.cpp

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


#include "openjaus/core/Managed.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace core
{

Managed::Managed() : Base(),
	toReady(),
	pause(),
	resetTransition(),
	shutdownTransition(),
	pushToEmergency(),
	popFromEmergency(),
	managementLoopback(),
	initializedTransition(),
	standby(),
	ready(),
	init(),
	shutdown(),
	emergency(),
	standbyReady()
{
	// Add Service Identification Data to implements list
	name = "Managed";
	
	model::Service *managementService = new model::Service();
	managementService->setName("Management");
	managementService->setUri("urn:jaus:jss:core:Management");
	managementService->setVersionMajor(1);
	managementService->setVersionMinor(1);
	this->implements->push_back(managementService);
	
	standby.setName("Standby");
	ready.setName("Ready");
	
	init.setName("Init");
	accessStateMachine.addState(init);
	accessStateMachine.setStartingState(&init);
	shutdown.setName("Shutdown");
	accessStateMachine.addState(shutdown);
	emergency.setName("Emergency");
	accessStateMachine.addState(emergency);

	toReady.setInterface(this);
	toReady.setTransportInterface(this);
	toReady.setEndState(&ready);
	toReady.setStartState(&standby);
	standby.addTransition(toReady);
	
	pause.setInterface(this);
	pause.setTransportInterface(this);
	pause.setEndState(&standby);
	pause.setStartState(&ready);
	ready.addTransition(pause);
	
	resetTransition.setInterface(this);
	resetTransition.setTransportInterface(this);
	resetTransition.setEndState(&init);
	resetTransition.setStartState(&controlled);
	controlled.addTransition(resetTransition);
	
	shutdownTransition.setInterface(this);
	shutdownTransition.setTransportInterface(this);
	shutdownTransition.setEndState(&shutdown);
	controlled.addTransition(shutdownTransition);
	
	pushToEmergency.setInterface(this);
	pushToEmergency.setTransportInterface(this);
	pushToEmergency.setEndState(&emergency);
	accessStateMachine.addDefaultStateTransition(pushToEmergency);
	
	popFromEmergency.setInterface(this);
	popFromEmergency.setTransportInterface(this);
	popFromEmergency.setStartState(&emergency);
	emergency.addTransition(popFromEmergency);
	
	managementLoopback.setInterface(this);
	managementLoopback.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(managementLoopback);
	
	initializedTransition.setInterface(this);
	initializedTransition.setTransportInterface(this);
	initializedTransition.setEndState(&notControlled);
	initializedTransition.setStartState(&init);
	init.addTransition(initializedTransition);
	
	standbyReady.setName("StandbyReady");
	standbyReady.addState(standby);
	standbyReady.addState(ready);
	standbyReady.setStartingState(&standby);
	controlled.addStateMachine(standbyReady);
	
    
    
	// Start of user code for Constructor:
	storedIds.clear();
	// End of user code
}

Managed::~Managed()
{
	// Start of user code for Destructor:
	// End of user code
}

bool Managed::storeID(SetEmergency *setEmergency)
{
	// Start of user code for action storeID(SetEmergency *setEmergency):
	storedIds.push_back(setEmergency->getSource());
	return true;
	// End of user code
}

bool Managed::deleteID(ClearEmergency *clearEmergency)
{
	// Start of user code for action deleteID(ClearEmergency *clearEmergency):
	for(size_t i = 0; i < storedIds.size(); i++)
	{
		if(storedIds[i] == clearEmergency->getSource())
		{
			storedIds.erase(storedIds.begin() + i);
			return true;
		}
	}
	return false;
	// End of user code
}

core::ReportStatus Managed::getReportStatus(QueryStatus *queryStatus)
{
	// Start of user code for action getReportStatus(QueryStatus *queryStatus):
	core::ReportStatus message;
	model::State *state = accessStateMachine.getCurrentState();

	if(state == &init)
	{
		message.setStatus(core::StatusEnumeration::INIT);
	}
	else if(state == &shutdown)
	{
		message.setStatus(core::StatusEnumeration::SHUTDOWN);
	}
	else if(state == &notControlled)
	{
		message.setStatus(core::StatusEnumeration::STANDBY);
	}
	else if(state == &emergency)
	{
		message.setStatus(core::StatusEnumeration::EMERGENCY);
	}
	else if(state == &controlled)
	{
		model::State *state = standbyReady.getCurrentState();
		if(state == &standby)
		{
			message.setStatus(core::StatusEnumeration::STANDBY);
		}
		else if(state == &ready)
		{
			message.setStatus(core::StatusEnumeration::READY);
		}
	}

	return message;
	// End of user code
}


bool Managed::isControllingClient(Reset *reset)
{
	// Start of user code for action isControllingClient(Reset *reset):
	if(controllerAddress == reset->getSource())
	{
		return true;
	}
	return false;

	// End of user code
}

bool Managed::isControllingClient(Shutdown *shutdown)
{
	// Start of user code for action isControllingClient(Shutdown *shutdown):
	if(controllerAddress == shutdown->getSource())
	{
		return true;
	}
	return false;
	// End of user code
}


bool Managed::isIDStored(ClearEmergency *clearEmergency)
{
	// Start of user code for action isIDStored(ClearEmergency *clearEmergency):
	for(size_t i = 0; i < storedIds.size(); i++)
	{
		if(storedIds[i] == clearEmergency->getSource())
		{
			return true;
		}
	}
	return false;

	// End of user code
}


// Start of user code for additional methods
void Managed::initialized()
{
	LOG_DEBUG("Managed component initialized");
	Initialized *initEvent = new Initialized();
	receiveThread.push(initEvent);
}
// End of user code

} // namespace component
} // namespace openjaus

