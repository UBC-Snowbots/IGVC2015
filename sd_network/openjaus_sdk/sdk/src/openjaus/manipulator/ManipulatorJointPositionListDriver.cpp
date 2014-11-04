/**
\file ManipulatorJointPositionListDriver.cpp

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


#include "openjaus/manipulator/ManipulatorJointPositionListDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

ManipulatorJointPositionListDriver::ManipulatorJointPositionListDriver() : Managed(),
	jointPositionListDriverDefaultLoop(),
	jointPositionListDriverControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "ManipulatorJointPositionListDriver";
	
	model::Service *manipulatorJointPositionListDriverService = new model::Service();
	manipulatorJointPositionListDriverService->setName("ManipulatorJointPositionListDriver");
	manipulatorJointPositionListDriverService->setUri("urn:jaus:jss:manipulator:ManipulatorJointPositionListDriver");
	manipulatorJointPositionListDriverService->setVersionMajor(1);
	manipulatorJointPositionListDriverService->setVersionMinor(0);
	this->implements->push_back(manipulatorJointPositionListDriverService);
	
	

	jointPositionListDriverDefaultLoop.setInterface(this);
	jointPositionListDriverDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(jointPositionListDriverDefaultLoop);
	
	jointPositionListDriverControlledLoop.setInterface(this);
	jointPositionListDriverControlledLoop.setTransportInterface(this);
	controlled.addTransition(jointPositionListDriverControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

ManipulatorJointPositionListDriver::~ManipulatorJointPositionListDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

bool ManipulatorJointPositionListDriver::executeTargetList(ExecuteList *executeList)
{
	// Start of user code for action executeTargetList(ExecuteList *executeList):
	return false;
	// End of user code
}

bool ManipulatorJointPositionListDriver::setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile)
{
	// Start of user code for action setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile):
	return false;
	// End of user code
}

bool ManipulatorJointPositionListDriver::setElement(model::Trigger *trigger)
{
	// Start of user code for action setElement(model::Trigger *trigger):
	return false;
	// End of user code
}

manipulator::ReportManipulatorSpecifications ManipulatorJointPositionListDriver::getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications)
{
	// Start of user code for action getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications):
	manipulator::ReportManipulatorSpecifications message;
	return message;
	// End of user code
}

manipulator::ReportJointMotionProfile ManipulatorJointPositionListDriver::getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile)
{
	// Start of user code for action getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile):
	manipulator::ReportJointMotionProfile message;
	return message;
	// End of user code
}

manipulator::ReportActiveElement ManipulatorJointPositionListDriver::getReportActiveElement(QueryActiveElement *queryActiveElement)
{
	// Start of user code for action getReportActiveElement(QueryActiveElement *queryActiveElement):
	manipulator::ReportActiveElement message;
	return message;
	// End of user code
}

manipulator::ReportCommandedJointPosition ManipulatorJointPositionListDriver::getReportCommandedJointPosition(QueryCommandedJointPosition *queryCommandedJointPosition)
{
	// Start of user code for action getReportCommandedJointPosition(QueryCommandedJointPosition *queryCommandedJointPosition):
	manipulator::ReportCommandedJointPosition message;
	return message;
	// End of user code
}


bool ManipulatorJointPositionListDriver::isControllingJointPositionListClient(SetJointMotionProfile *setJointMotionProfile)
{
	// Start of user code for action isControllingJointPositionListClient(SetJointMotionProfile *setJointMotionProfile):
	return false;
	// End of user code
}

bool ManipulatorJointPositionListDriver::isControllingJointPositionListClient(ExecuteList *executeList)
{
	// Start of user code for action isControllingJointPositionListClient(ExecuteList *executeList):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

