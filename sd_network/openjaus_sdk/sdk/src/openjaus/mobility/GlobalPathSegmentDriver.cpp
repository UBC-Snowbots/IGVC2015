/**
\file GlobalPathSegmentDriver.cpp

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


#include "openjaus/mobility/GlobalPathSegmentDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

GlobalPathSegmentDriver::GlobalPathSegmentDriver() : ListManaged(),
	gpsdDefaultLoop(),
	gpsdControlledLoop(),
	gpsdReadyLoop()
{
	// Add Service Identification Data to implements list
	name = "GlobalPathSegmentDriver";
	
	model::Service *globalPathSegmentDriverService = new model::Service();
	globalPathSegmentDriverService->setName("GlobalPathSegmentDriver");
	globalPathSegmentDriverService->setUri("urn:jaus:jss:mobility:GlobalPathSegmentDriver");
	globalPathSegmentDriverService->setVersionMajor(1);
	globalPathSegmentDriverService->setVersionMinor(0);
	this->implements->push_back(globalPathSegmentDriverService);
	
	

	gpsdDefaultLoop.setInterface(this);
	gpsdDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(gpsdDefaultLoop);
	
	gpsdControlledLoop.setInterface(this);
	gpsdControlledLoop.setTransportInterface(this);
	controlled.addTransition(gpsdControlledLoop);
	
	gpsdReadyLoop.setInterface(this);
	gpsdReadyLoop.setTransportInterface(this);
	ready.addTransition(gpsdReadyLoop);
	
    
	ready.addExitAction(&GlobalPathSegmentDriver::resetGpsdTravelSpeed, this);
    
	// Start of user code for Constructor:
	// End of user code
}

GlobalPathSegmentDriver::~GlobalPathSegmentDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

bool GlobalPathSegmentDriver::setGpsdElement(SetElement *setElement)
{
	// Start of user code for action setGpsdElement(SetElement *setElement):
	return false;
	// End of user code
}

bool GlobalPathSegmentDriver::executeGlobalPathSegmentList(ExecuteList *executeList)
{
	// Start of user code for action executeGlobalPathSegmentList(ExecuteList *executeList):
	return false;
	// End of user code
}

bool GlobalPathSegmentDriver::modifyGpsdTravelSpeed(ExecuteList *executeList)
{
	// Start of user code for action modifyGpsdTravelSpeed(ExecuteList *executeList):
	return false;
	// End of user code
}

mobility::ReportGlobalPathSegment GlobalPathSegmentDriver::getReportGlobalPathSegment(QueryGlobalPathSegment *queryGlobalPathSegment)
{
	// Start of user code for action getReportGlobalPathSegment(QueryGlobalPathSegment *queryGlobalPathSegment):
	mobility::ReportGlobalPathSegment message;
	return message;
	// End of user code
}

mobility::ReportTravelSpeed GlobalPathSegmentDriver::getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed)
{
	// Start of user code for action getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed):
	mobility::ReportTravelSpeed message;
	return message;
	// End of user code
}

mobility::ReportActiveElement GlobalPathSegmentDriver::getReportActiveElement(QueryActiveElement *queryActiveElement)
{
	// Start of user code for action getReportActiveElement(QueryActiveElement *queryActiveElement):
	mobility::ReportActiveElement message;
	return message;
	// End of user code
}

mobility::ConfirmElementRequest GlobalPathSegmentDriver::getConfirmElementRequest(SetElement *setElement)
{
	// Start of user code for action getConfirmElementRequest(SetElement *setElement):
	mobility::ConfirmElementRequest message;
	return message;
	// End of user code
}

mobility::RejectElementRequest GlobalPathSegmentDriver::getRejectElementRequest(SetElement *setElement)
{
	// Start of user code for action getRejectElementRequest(SetElement *setElement):
	mobility::RejectElementRequest message;
	return message;
	// End of user code
}

void GlobalPathSegmentDriver::resetGpsdTravelSpeed()
{
	// Start of user code for action resetGpsdTravelSpeed():
	// End of user code
}


bool GlobalPathSegmentDriver::isControllingGpsdClient(SetElement *setElement)
{
	// Start of user code for action isControllingGpsdClient(SetElement *setElement):
	return false;
	// End of user code
}

bool GlobalPathSegmentDriver::isControllingGpsdClient(ExecuteList *executeList)
{
	// Start of user code for action isControllingGpsdClient(ExecuteList *executeList):
	return false;
	// End of user code
}


bool GlobalPathSegmentDriver::gpsdSegmentExists(QueryGlobalPathSegment *queryGlobalPathSegment)
{
	// Start of user code for action gpsdSegmentExists(QueryGlobalPathSegment *queryGlobalPathSegment):
	return false;
	// End of user code
}


bool GlobalPathSegmentDriver::gpsdElementExists(ExecuteList *executeList)
{
	// Start of user code for action gpsdElementExists(ExecuteList *executeList):
	return false;
	// End of user code
}


bool GlobalPathSegmentDriver::isValidGpsdElementRequest(SetElement *setElement)
{
	// Start of user code for action isValidGpsdElementRequest(SetElement *setElement):
	return false;
	// End of user code
}


bool GlobalPathSegmentDriver::isGpsdElementSupported(SetElement *setElement)
{
	// Start of user code for action isGpsdElementSupported(SetElement *setElement):
	return false;
	// End of user code
}


bool GlobalPathSegmentDriver::gpsdElementSpecified(ExecuteList *executeList)
{
	// Start of user code for action gpsdElementSpecified(ExecuteList *executeList):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

