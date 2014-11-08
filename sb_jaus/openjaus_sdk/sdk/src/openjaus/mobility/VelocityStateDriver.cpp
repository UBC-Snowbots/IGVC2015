/**
\file VelocityStateDriver.cpp

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


#include "openjaus/mobility/VelocityStateDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

VelocityStateDriver::VelocityStateDriver() : Managed(),
	vsdDefaultLoop(),
	vsdControlledLoop(),
	vsdReadyLoop()
{
	// Add Service Identification Data to implements list
	name = "VelocityStateDriver";
	
	model::Service *velocityStateDriverService = new model::Service();
	velocityStateDriverService->setName("VelocityStateDriver");
	velocityStateDriverService->setUri("urn:jaus:jss:mobility:VelocityStateDriver");
	velocityStateDriverService->setVersionMajor(1);
	velocityStateDriverService->setVersionMinor(0);
	this->implements->push_back(velocityStateDriverService);
	
	

	vsdDefaultLoop.setInterface(this);
	vsdDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(vsdDefaultLoop);
	
	vsdControlledLoop.setInterface(this);
	vsdControlledLoop.setTransportInterface(this);
	controlled.addTransition(vsdControlledLoop);
	
	vsdReadyLoop.setInterface(this);
	vsdReadyLoop.setTransportInterface(this);
	ready.addTransition(vsdReadyLoop);
	
    
	ready.addExitAction(&VelocityStateDriver::resetCmdVelocityToDefault, this);
    
	// Start of user code for Constructor:
	// End of user code
}

VelocityStateDriver::~VelocityStateDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

void VelocityStateDriver::resetCmdVelocityToDefault()
{
	// Start of user code for action resetCmdVelocityToDefault():
	// End of user code
}

mobility::ReportVelocityCommand VelocityStateDriver::getReportVelocityCommand(QueryVelocityCommand *queryVelocityCommand)
{
	// Start of user code for action getReportVelocityCommand(QueryVelocityCommand *queryVelocityCommand):
	mobility::ReportVelocityCommand message;
	return message;
	// End of user code
}

mobility::ReportAccelerationLimit VelocityStateDriver::getReportAccelerationLimit(QueryAccelerationLimit *queryAccelerationLimit)
{
	// Start of user code for action getReportAccelerationLimit(QueryAccelerationLimit *queryAccelerationLimit):
	mobility::ReportAccelerationLimit message;
	return message;
	// End of user code
}

bool VelocityStateDriver::setCurrentVelocityCommand(SetVelocityCommand *setVelocityCommand)
{
	// Start of user code for action setCurrentVelocityCommand(SetVelocityCommand *setVelocityCommand):
	return false;
	// End of user code
}

bool VelocityStateDriver::setVelocityLimit(SetVelocityCommand *setVelocityCommand)
{
	// Start of user code for action setVelocityLimit(SetVelocityCommand *setVelocityCommand):
	return false;
	// End of user code
}

bool VelocityStateDriver::setAccelerationLimit(SetAccelerationLimit *setAccelerationLimit)
{
	// Start of user code for action setAccelerationLimit(SetAccelerationLimit *setAccelerationLimit):
	return false;
	// End of user code
}


bool VelocityStateDriver::isControllingVsdClient(SetVelocityCommand *setVelocityCommand)
{
	// Start of user code for action isControllingVsdClient(SetVelocityCommand *setVelocityCommand):
	return false;
	// End of user code
}

bool VelocityStateDriver::isControllingVsdClient(SetAccelerationLimit *setAccelerationLimit)
{
	// Start of user code for action isControllingVsdClient(SetAccelerationLimit *setAccelerationLimit):
	return false;
	// End of user code
}


bool VelocityStateDriver::isCurrentVelocityCmd(SetVelocityCommand *setVelocityCommand)
{
	// Start of user code for action isCurrentVelocityCmd(SetVelocityCommand *setVelocityCommand):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

