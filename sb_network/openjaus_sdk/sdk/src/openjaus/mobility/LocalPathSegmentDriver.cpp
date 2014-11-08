/**
\file LocalPathSegmentDriver.cpp

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


#include "openjaus/mobility/LocalPathSegmentDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

LocalPathSegmentDriver::LocalPathSegmentDriver() : ListManaged(),
	lpsdDefaultLoop(),
	lpsdControlledLoop(),
	lpsdReadyLoop()
{
	// Add Service Identification Data to implements list
	name = "LocalPathSegmentDriver";
	
	model::Service *localPathSegmentDriverService = new model::Service();
	localPathSegmentDriverService->setName("LocalPathSegmentDriver");
	localPathSegmentDriverService->setUri("urn:jaus:jss:mobility:LocalPathSegmentDriver");
	localPathSegmentDriverService->setVersionMajor(1);
	localPathSegmentDriverService->setVersionMinor(0);
	this->implements->push_back(localPathSegmentDriverService);
	
	

	lpsdDefaultLoop.setInterface(this);
	lpsdDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(lpsdDefaultLoop);
	
	lpsdControlledLoop.setInterface(this);
	lpsdControlledLoop.setTransportInterface(this);
	controlled.addTransition(lpsdControlledLoop);
	
	lpsdReadyLoop.setInterface(this);
	lpsdReadyLoop.setTransportInterface(this);
	ready.addTransition(lpsdReadyLoop);
	
    
	ready.addExitAction(&LocalPathSegmentDriver::resetLpsdTravelSpeed, this);
    
	// Start of user code for Constructor:
	// End of user code
}

LocalPathSegmentDriver::~LocalPathSegmentDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

bool LocalPathSegmentDriver::setLpsdElement(SetElement *setElement)
{
	// Start of user code for action setLpsdElement(SetElement *setElement):
	return false;
	// End of user code
}

bool LocalPathSegmentDriver::executeLocalPathSegmentList(ExecuteList *executeList)
{
	// Start of user code for action executeLocalPathSegmentList(ExecuteList *executeList):
	return false;
	// End of user code
}

bool LocalPathSegmentDriver::modifyLpsdTravelSpeed(ExecuteList *executeList)
{
	// Start of user code for action modifyLpsdTravelSpeed(ExecuteList *executeList):
	return false;
	// End of user code
}

mobility::ReportLocalPathSegment LocalPathSegmentDriver::getReportLocalPathSegment(QueryLocalPathSegment *queryLocalPathSegment)
{
	// Start of user code for action getReportLocalPathSegment(QueryLocalPathSegment *queryLocalPathSegment):
	mobility::ReportLocalPathSegment message;
	return message;
	// End of user code
}

mobility::ReportTravelSpeed LocalPathSegmentDriver::getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed)
{
	// Start of user code for action getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed):
	mobility::ReportTravelSpeed message;
	return message;
	// End of user code
}

mobility::ReportActiveElement LocalPathSegmentDriver::getReportActiveElement(QueryActiveElement *queryActiveElement)
{
	// Start of user code for action getReportActiveElement(QueryActiveElement *queryActiveElement):
	mobility::ReportActiveElement message;
	return message;
	// End of user code
}

mobility::ConfirmElementRequest LocalPathSegmentDriver::getConfirmElementRequest(SetElement *setElement)
{
	// Start of user code for action getConfirmElementRequest(SetElement *setElement):
	mobility::ConfirmElementRequest message;
	return message;
	// End of user code
}

mobility::RejectElementRequest LocalPathSegmentDriver::getRejectElementRequest(SetElement *setElement)
{
	// Start of user code for action getRejectElementRequest(SetElement *setElement):
	mobility::RejectElementRequest message;
	return message;
	// End of user code
}

void LocalPathSegmentDriver::resetLpsdTravelSpeed()
{
	// Start of user code for action resetLpsdTravelSpeed():
	// End of user code
}


bool LocalPathSegmentDriver::isControllingLpsdClient(SetElement *setElement)
{
	// Start of user code for action isControllingLpsdClient(SetElement *setElement):
	return false;
	// End of user code
}

bool LocalPathSegmentDriver::isControllingLpsdClient(ExecuteList *executeList)
{
	// Start of user code for action isControllingLpsdClient(ExecuteList *executeList):
	return false;
	// End of user code
}


bool LocalPathSegmentDriver::lpsdSegmentExists(QueryLocalPathSegment *queryLocalPathSegment)
{
	// Start of user code for action lpsdSegmentExists(QueryLocalPathSegment *queryLocalPathSegment):
	return false;
	// End of user code
}


bool LocalPathSegmentDriver::lpsdElementExists(ExecuteList *executeList)
{
	// Start of user code for action lpsdElementExists(ExecuteList *executeList):
	return false;
	// End of user code
}


bool LocalPathSegmentDriver::isValidLpsdElementRequest(SetElement *setElement)
{
	// Start of user code for action isValidLpsdElementRequest(SetElement *setElement):
	return false;
	// End of user code
}


bool LocalPathSegmentDriver::isLpsdElementSupported(SetElement *setElement)
{
	// Start of user code for action isLpsdElementSupported(SetElement *setElement):
	return false;
	// End of user code
}


bool LocalPathSegmentDriver::lpsdElementSpecified(ExecuteList *executeList)
{
	// Start of user code for action lpsdElementSpecified(ExecuteList *executeList):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

