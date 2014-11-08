/**
\file LocalWaypointListDriver.cpp

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


#include "openjaus/mobility/LocalWaypointListDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

LocalWaypointListDriver::LocalWaypointListDriver() : ListManaged(),
	lwldDefaultLoop(),
	lwldControlledLoop(),
	lwldReadyLoop()
{
	// Add Service Identification Data to implements list
	name = "LocalWaypointListDriver";
	
	model::Service *localWaypointListDriverService = new model::Service();
	localWaypointListDriverService->setName("LocalWaypointListDriver");
	localWaypointListDriverService->setUri("urn:jaus:jss:mobility:LocalWaypointListDriver");
	localWaypointListDriverService->setVersionMajor(1);
	localWaypointListDriverService->setVersionMinor(0);
	this->implements->push_back(localWaypointListDriverService);
	
	

	lwldDefaultLoop.setInterface(this);
	lwldDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(lwldDefaultLoop);
	
	lwldControlledLoop.setInterface(this);
	lwldControlledLoop.setTransportInterface(this);
	lwldControlledLoop.setStartState(&controlled);
	
	lwldReadyLoop.setInterface(this);
	lwldReadyLoop.setTransportInterface(this);
	lwldReadyLoop.setStartState(&ready);
	
    
	ready.addExitAction(&LocalWaypointListDriver::resetLwldTravelSpeed, this);
    
	// Start of user code for Constructor:
	// End of user code
}

LocalWaypointListDriver::~LocalWaypointListDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

void LocalWaypointListDriver::resetLwldTravelSpeed()
{
	// Start of user code for action resetLwldTravelSpeed():
	// End of user code
}

bool LocalWaypointListDriver::setLocalWaypointElement(SetElement *setElement)
{
	// Start of user code for action setLocalWaypointElement(SetElement *setElement):
	return false;
	// End of user code
}

bool LocalWaypointListDriver::executeLocalWaypointList(ExecuteList *executeList)
{
	// Start of user code for action executeLocalWaypointList(ExecuteList *executeList):
	return false;
	// End of user code
}

bool LocalWaypointListDriver::modifyLwldTravelSpeed(ExecuteList *executeList)
{
	// Start of user code for action modifyLwldTravelSpeed(ExecuteList *executeList):
	return false;
	// End of user code
}

mobility::ReportLocalWaypoint LocalWaypointListDriver::getReportLocalWaypoint(QueryLocalWaypoint *queryLocalWaypoint)
{
	// Start of user code for action getReportLocalWaypoint(QueryLocalWaypoint *queryLocalWaypoint):
	mobility::ReportLocalWaypoint message;
	return message;
	// End of user code
}

mobility::ReportTravelSpeed LocalWaypointListDriver::getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed)
{
	// Start of user code for action getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed):
	mobility::ReportTravelSpeed message;
	return message;
	// End of user code
}

mobility::ReportActiveElement LocalWaypointListDriver::getReportActiveElement(QueryActiveElement *queryActiveElement)
{
	// Start of user code for action getReportActiveElement(QueryActiveElement *queryActiveElement):
	mobility::ReportActiveElement message;
	return message;
	// End of user code
}

mobility::ConfirmElementRequest LocalWaypointListDriver::getConfirmElementRequest(SetElement *setElement)
{
	// Start of user code for action getConfirmElementRequest(SetElement *setElement):
	mobility::ConfirmElementRequest message;
	return message;
	// End of user code
}

mobility::RejectElementRequest LocalWaypointListDriver::getRejectElementRequest(SetElement *setElement)
{
	// Start of user code for action getRejectElementRequest(SetElement *setElement):
	mobility::RejectElementRequest message;
	return message;
	// End of user code
}


bool LocalWaypointListDriver::isControllingLwldClient(SetElement *setElement)
{
	// Start of user code for action isControllingLwldClient(SetElement *setElement):
	return false;
	// End of user code
}

bool LocalWaypointListDriver::isControllingLwldClient(ExecuteList *executeList)
{
	// Start of user code for action isControllingLwldClient(ExecuteList *executeList):
	return false;
	// End of user code
}


bool LocalWaypointListDriver::lwldWaypointExists(QueryLocalWaypoint *queryLocalWaypoint)
{
	// Start of user code for action lwldWaypointExists(QueryLocalWaypoint *queryLocalWaypoint):
	return false;
	// End of user code
}


bool LocalWaypointListDriver::lwldElementExists(ExecuteList *executeList)
{
	// Start of user code for action lwldElementExists(ExecuteList *executeList):
	return false;
	// End of user code
}


bool LocalWaypointListDriver::isValidLwldElementRequest(SetElement *setElement)
{
	// Start of user code for action isValidLwldElementRequest(SetElement *setElement):
	return false;
	// End of user code
}


bool LocalWaypointListDriver::isLwldElementSupported(SetElement *setElement)
{
	// Start of user code for action isLwldElementSupported(SetElement *setElement):
	return false;
	// End of user code
}


bool LocalWaypointListDriver::lwldElementSpecified(model::Trigger *trigger)
{
	// Start of user code for action lwldElementSpecified(model::Trigger *trigger):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

