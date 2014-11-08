/**
\file ManipulatorEndEffectorVelocityStateDriver.cpp

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


#include "openjaus/manipulator/ManipulatorEndEffectorVelocityStateDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

ManipulatorEndEffectorVelocityStateDriver::ManipulatorEndEffectorVelocityStateDriver() : Managed(),
	endEffectorVelocityStateDriverDefaultLoop(),
	endEffectorVelocityStateDriverControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "ManipulatorEndEffectorVelocityStateDriver";
	
	model::Service *manipulatorEndEffectorVelocityStateDriverService = new model::Service();
	manipulatorEndEffectorVelocityStateDriverService->setName("ManipulatorEndEffectorVelocityStateDriver");
	manipulatorEndEffectorVelocityStateDriverService->setUri("urn:jaus:jss:manipulator:ManipulatorEndEffectorVelocityStateDriver");
	manipulatorEndEffectorVelocityStateDriverService->setVersionMajor(1);
	manipulatorEndEffectorVelocityStateDriverService->setVersionMinor(0);
	this->implements->push_back(manipulatorEndEffectorVelocityStateDriverService);
	
	

	endEffectorVelocityStateDriverDefaultLoop.setInterface(this);
	endEffectorVelocityStateDriverDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(endEffectorVelocityStateDriverDefaultLoop);
	
	endEffectorVelocityStateDriverControlledLoop.setInterface(this);
	endEffectorVelocityStateDriverControlledLoop.setTransportInterface(this);
	controlled.addTransition(endEffectorVelocityStateDriverControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

ManipulatorEndEffectorVelocityStateDriver::~ManipulatorEndEffectorVelocityStateDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

bool ManipulatorEndEffectorVelocityStateDriver::setEndEffectorVelocityState(SetEndEffectorVelocityState *setEndEffectorVelocityState)
{
	// Start of user code for action setEndEffectorVelocityState(SetEndEffectorVelocityState *setEndEffectorVelocityState):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorVelocityStateDriver::setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile)
{
	// Start of user code for action setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorVelocityStateDriver::setToolOffset(SetToolOffset *setToolOffset)
{
	// Start of user code for action setToolOffset(SetToolOffset *setToolOffset):
	return false;
	// End of user code
}

manipulator::ReportManipulatorSpecifications ManipulatorEndEffectorVelocityStateDriver::getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications)
{
	// Start of user code for action getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications):
	manipulator::ReportManipulatorSpecifications message;
	return message;
	// End of user code
}

manipulator::ReportToolOffset ManipulatorEndEffectorVelocityStateDriver::getReportToolOffset(QueryToolOffset *queryToolOffset)
{
	// Start of user code for action getReportToolOffset(QueryToolOffset *queryToolOffset):
	manipulator::ReportToolOffset message;
	return message;
	// End of user code
}

manipulator::ReportCommandedEndEffectorVelocityState ManipulatorEndEffectorVelocityStateDriver::getReportCommandedEndEffectorVelocityState(QueryCommandedEndEffectorVelocityState *queryCommandedEndEffectorVelocityState)
{
	// Start of user code for action getReportCommandedEndEffectorVelocityState(QueryCommandedEndEffectorVelocityState *queryCommandedEndEffectorVelocityState):
	manipulator::ReportCommandedEndEffectorVelocityState message;
	return message;
	// End of user code
}

manipulator::ReportJointMotionProfile ManipulatorEndEffectorVelocityStateDriver::getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile)
{
	// Start of user code for action getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile):
	manipulator::ReportJointMotionProfile message;
	return message;
	// End of user code
}


bool ManipulatorEndEffectorVelocityStateDriver::isControllingEndEffectorVelocityStateClient(SetJointMotionProfile *setJointMotionProfile)
{
	// Start of user code for action isControllingEndEffectorVelocityStateClient(SetJointMotionProfile *setJointMotionProfile):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorVelocityStateDriver::isControllingEndEffectorVelocityStateClient(SetToolOffset *setToolOffset)
{
	// Start of user code for action isControllingEndEffectorVelocityStateClient(SetToolOffset *setToolOffset):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorVelocityStateDriver::isControllingEndEffectorVelocityStateClient(SetEndEffectorVelocityState *setEndEffectorVelocityState)
{
	// Start of user code for action isControllingEndEffectorVelocityStateClient(SetEndEffectorVelocityState *setEndEffectorVelocityState):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

