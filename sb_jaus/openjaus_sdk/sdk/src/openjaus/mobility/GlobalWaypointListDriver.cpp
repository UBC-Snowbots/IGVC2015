/**
\file GlobalWaypointListDriver.cpp

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


#include "openjaus/mobility/GlobalWaypointListDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

GlobalWaypointListDriver::GlobalWaypointListDriver() : ListManaged(),
	gwldDefaultLoop(),
	gwldControlledLoop(),
	gwldReadyLoop()
{
	// Add Service Identification Data to implements list
	name = "GlobalWaypointListDriver";
	
	model::Service *globalWaypointListDriverService = new model::Service();
	globalWaypointListDriverService->setName("GlobalWaypointListDriver");
	globalWaypointListDriverService->setUri("urn:jaus:jss:mobility:GlobalWaypointListDriver");
	globalWaypointListDriverService->setVersionMajor(1);
	globalWaypointListDriverService->setVersionMinor(0);
	this->implements->push_back(globalWaypointListDriverService);
	
	

	gwldDefaultLoop.setInterface(this);
	gwldDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(gwldDefaultLoop);
	
	gwldControlledLoop.setInterface(this);
	gwldControlledLoop.setTransportInterface(this);
	controlled.addTransition(gwldControlledLoop);
	
	gwldReadyLoop.setInterface(this);
	gwldReadyLoop.setTransportInterface(this);
	ready.addTransition(gwldReadyLoop);
	
    
	ready.addExitAction(&GlobalWaypointListDriver::resetGwldTravelSpeed, this);
    
	// Start of user code for Constructor:
	// End of user code
}

GlobalWaypointListDriver::~GlobalWaypointListDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

void GlobalWaypointListDriver::resetGwldTravelSpeed()
{
	// Start of user code for action resetGwldTravelSpeed():
	// End of user code
}

bool GlobalWaypointListDriver::setGlobalWaypointElement(SetElement *setElement)
{
	// Start of user code for action setGlobalWaypointElement(SetElement *setElement):
	return false;
	// End of user code
}

bool GlobalWaypointListDriver::executeGlobalWaypointList(ExecuteList *executeList)
{
	// Start of user code for action executeGlobalWaypointList(ExecuteList *executeList):
	return false;
	// End of user code
}

bool GlobalWaypointListDriver::modifyGwldTravelSpeed(ExecuteList *executeList)
{
	// Start of user code for action modifyGwldTravelSpeed(ExecuteList *executeList):
	return false;
	// End of user code
}

mobility::ReportGlobalWaypoint GlobalWaypointListDriver::getReportGlobalWaypoint(QueryGlobalWaypoint *queryGlobalWaypoint)
{
	// Start of user code for action getReportGlobalWaypoint(QueryGlobalWaypoint *queryGlobalWaypoint):
	mobility::ReportGlobalWaypoint message;
	return message;
	// End of user code
}

mobility::ReportTravelSpeed GlobalWaypointListDriver::getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed)
{
	// Start of user code for action getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed):
	mobility::ReportTravelSpeed message;
	return message;
	// End of user code
}

mobility::ReportActiveElement GlobalWaypointListDriver::getReportActiveElement(QueryActiveElement *queryActiveElement)
{
	// Start of user code for action getReportActiveElement(QueryActiveElement *queryActiveElement):
	mobility::ReportActiveElement message;
	return message;
	// End of user code
}

mobility::ConfirmElementRequest GlobalWaypointListDriver::getConfirmElementRequest(SetElement *setElement)
{
	// Start of user code for action getConfirmElementRequest(SetElement *setElement):
	mobility::ConfirmElementRequest message;
	return message;
	// End of user code
}

mobility::RejectElementRequest GlobalWaypointListDriver::getRejectElementRequest(SetElement *setElement)
{
	// Start of user code for action getRejectElementRequest(SetElement *setElement):
	mobility::RejectElementRequest message;
	return message;
	// End of user code
}


bool GlobalWaypointListDriver::isControllingGwldClient(SetElement *setElement)
{
	// Start of user code for action isControllingGwldClient(SetElement *setElement):
	return false;
	// End of user code
}

bool GlobalWaypointListDriver::isControllingGwldClient(ExecuteList *executeList)
{
	// Start of user code for action isControllingGwldClient(ExecuteList *executeList):
	return false;
	// End of user code
}


bool GlobalWaypointListDriver::gwldWaypointExists(QueryGlobalWaypoint *queryGlobalWaypoint)
{
	// Start of user code for action gwldWaypointExists(QueryGlobalWaypoint *queryGlobalWaypoint):
	return false;
	// End of user code
}

bool GlobalWaypointListDriver::gwldWaypointExists(ExecuteList *executeList)
{
	// Start of user code for action gwldWaypointExists(ExecuteList *executeList):
	return false;
	// End of user code
}


bool GlobalWaypointListDriver::gwldElementExists(model::Trigger *trigger)
{
	// Start of user code for action gwldElementExists(model::Trigger *trigger):
	return false;
	// End of user code
}


bool GlobalWaypointListDriver::isValidGwldElementRequest(SetElement *setElement)
{
	// Start of user code for action isValidGwldElementRequest(SetElement *setElement):
	return false;
	// End of user code
}


bool GlobalWaypointListDriver::isGwldElementSupported(SetElement *setElement)
{
	// Start of user code for action isGwldElementSupported(SetElement *setElement):
	return false;
	// End of user code
}


bool GlobalWaypointListDriver::gwldElementSpecified(ExecuteList *executeList)
{
	// Start of user code for action gwldElementSpecified(ExecuteList *executeList):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

