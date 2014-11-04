/**
\file GlobalWaypointDriver.cpp

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


#include "openjaus/mobility/GlobalWaypointDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

GlobalWaypointDriver::GlobalWaypointDriver() : Managed(),
	gwdDefaultLoop(),
	gwdReadyLoop(),
	gwdControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "GlobalWaypointDriver";
	
	model::Service *globalWaypointDriverService = new model::Service();
	globalWaypointDriverService->setName("GlobalWaypointDriver");
	globalWaypointDriverService->setUri("urn:jaus:jss:mobility:GlobalWaypointDriver");
	globalWaypointDriverService->setVersionMajor(1);
	globalWaypointDriverService->setVersionMinor(0);
	this->implements->push_back(globalWaypointDriverService);
	
	

	gwdDefaultLoop.setInterface(this);
	gwdDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(gwdDefaultLoop);
	
	gwdReadyLoop.setInterface(this);
	gwdReadyLoop.setTransportInterface(this);
	ready.addTransition(gwdReadyLoop);
	
	gwdControlledLoop.setInterface(this);
	gwdControlledLoop.setTransportInterface(this);
	controlled.addTransition(gwdControlledLoop);
	
    
	ready.addExitAction(&GlobalWaypointDriver::resetGwdTravelSpeed, this);
    
	// Start of user code for Constructor:
	// End of user code
}

GlobalWaypointDriver::~GlobalWaypointDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

mobility::ReportGlobalWaypoint GlobalWaypointDriver::getReportGlobalWaypoint(QueryGlobalWaypoint *queryGlobalWaypoint)
{
	// Start of user code for action getReportGlobalWaypoint(QueryGlobalWaypoint *queryGlobalWaypoint):
	mobility::ReportGlobalWaypoint message;
	return message;
	// End of user code
}

mobility::ReportTravelSpeed GlobalWaypointDriver::getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed)
{
	// Start of user code for action getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed):
	mobility::ReportTravelSpeed message;
	return message;
	// End of user code
}

bool GlobalWaypointDriver::setGwdTravelSpeed(SetTravelSpeed *setTravelSpeed)
{
	// Start of user code for action setGwdTravelSpeed(SetTravelSpeed *setTravelSpeed):
	return false;
	// End of user code
}

bool GlobalWaypointDriver::setGlobalWaypoint(SetGlobalWaypoint *setGlobalWaypoint)
{
	// Start of user code for action setGlobalWaypoint(SetGlobalWaypoint *setGlobalWaypoint):
	return false;
	// End of user code
}

void GlobalWaypointDriver::resetGwdTravelSpeed()
{
	// Start of user code for action resetGwdTravelSpeed():
	// End of user code
}


bool GlobalWaypointDriver::waypointExists(QueryGlobalWaypoint *queryGlobalWaypoint)
{
	// Start of user code for action waypointExists(QueryGlobalWaypoint *queryGlobalWaypoint):
	return false;
	// End of user code
}

bool GlobalWaypointDriver::waypointExists(SetTravelSpeed *setTravelSpeed)
{
	// Start of user code for action waypointExists(SetTravelSpeed *setTravelSpeed):
	return false;
	// End of user code
}

bool GlobalWaypointDriver::waypointExists(SetGlobalWaypoint *setGlobalWaypoint)
{
	// Start of user code for action waypointExists(SetGlobalWaypoint *setGlobalWaypoint):
	return false;
	// End of user code
}


bool GlobalWaypointDriver::isControllingGwdClient(SetTravelSpeed *setTravelSpeed)
{
	// Start of user code for action isControllingGwdClient(SetTravelSpeed *setTravelSpeed):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

