/**
\file PanTiltJointPositionDriver.cpp

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


#include "openjaus/manipulator/PanTiltJointPositionDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

PanTiltJointPositionDriver::PanTiltJointPositionDriver() : Base(),
	panTiltPositionDriverDefaultLoop(),
	panTiltPositionDriverControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "PanTiltJointPositionDriver";
	
	model::Service *panTiltJointPositionDriverService = new model::Service();
	panTiltJointPositionDriverService->setName("PanTiltJointPositionDriver");
	panTiltJointPositionDriverService->setUri("urn:jaus:jss:manipulator:PanTiltJointPositionDriver");
	panTiltJointPositionDriverService->setVersionMajor(1);
	panTiltJointPositionDriverService->setVersionMinor(0);
	this->implements->push_back(panTiltJointPositionDriverService);
	
	

	panTiltPositionDriverDefaultLoop.setInterface(this);
	panTiltPositionDriverDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(panTiltPositionDriverDefaultLoop);
	
	panTiltPositionDriverControlledLoop.setInterface(this);
	panTiltPositionDriverControlledLoop.setTransportInterface(this);
	controlled.addTransition(panTiltPositionDriverControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

PanTiltJointPositionDriver::~PanTiltJointPositionDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

bool PanTiltJointPositionDriver::setPanTiltJointPosition(SetPanTiltJointPosition *setPanTiltJointPosition)
{
	// Start of user code for action setPanTiltJointPosition(SetPanTiltJointPosition *setPanTiltJointPosition):
	return false;
	// End of user code
}

bool PanTiltJointPositionDriver::setPanTiltMotionProfile(SetPanTiltMotionProfile *setPanTiltMotionProfile)
{
	// Start of user code for action setPanTiltMotionProfile(SetPanTiltMotionProfile *setPanTiltMotionProfile):
	return false;
	// End of user code
}

manipulator::ReportPanTiltSpecifications PanTiltJointPositionDriver::getReportPanTiltSpecifications(QueryPanTiltSpecifications *queryPanTiltSpecifications)
{
	// Start of user code for action getReportPanTiltSpecifications(QueryPanTiltSpecifications *queryPanTiltSpecifications):
	manipulator::ReportPanTiltSpecifications message;
	return message;
	// End of user code
}

manipulator::ReportCommandedPanTiltJointPosition PanTiltJointPositionDriver::getReportCommandedPanTiltJointPosition(QueryCommandedPanTiltJointPosition *queryCommandedPanTiltJointPosition)
{
	// Start of user code for action getReportCommandedPanTiltJointPosition(QueryCommandedPanTiltJointPosition *queryCommandedPanTiltJointPosition):
	manipulator::ReportCommandedPanTiltJointPosition message;
	return message;
	// End of user code
}

manipulator::ReportPanTiltMotionProfile PanTiltJointPositionDriver::getReportPanTiltMotionProfile(QueryPanTiltMotionProfile *queryPanTiltMotionProfile)
{
	// Start of user code for action getReportPanTiltMotionProfile(QueryPanTiltMotionProfile *queryPanTiltMotionProfile):
	manipulator::ReportPanTiltMotionProfile message;
	return message;
	// End of user code
}


bool PanTiltJointPositionDriver::isControllingPanTiltJointPositionClient(SetPanTiltJointPosition *setPanTiltJointPosition)
{
	// Start of user code for action isControllingPanTiltJointPositionClient(SetPanTiltJointPosition *setPanTiltJointPosition):
	return false;
	// End of user code
}

bool PanTiltJointPositionDriver::isControllingPanTiltJointPositionClient(SetPanTiltMotionProfile *setPanTiltMotionProfile)
{
	// Start of user code for action isControllingPanTiltJointPositionClient(SetPanTiltMotionProfile *setPanTiltMotionProfile):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

