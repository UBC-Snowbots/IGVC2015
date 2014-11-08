/**
\file ManipulatorEndEffectorPoseDriver.cpp

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


#include "openjaus/manipulator/ManipulatorEndEffectorPoseDriver.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

ManipulatorEndEffectorPoseDriver::ManipulatorEndEffectorPoseDriver() : Managed(),
	endEffectorPoseDriverDefaultLoop(),
	endEffectorPoseDriverControlledLoop()
{
	// Add Service Identification Data to implements list
	name = "ManipulatorEndEffectorPoseDriver";
	
	model::Service *manipulatorEndEffectorPoseDriverService = new model::Service();
	manipulatorEndEffectorPoseDriverService->setName("ManipulatorEndEffectorPoseDriver");
	manipulatorEndEffectorPoseDriverService->setUri("urn:jaus:jss:manipulator:ManipulatorEndEffectorPoseDriver");
	manipulatorEndEffectorPoseDriverService->setVersionMajor(1);
	manipulatorEndEffectorPoseDriverService->setVersionMinor(0);
	this->implements->push_back(manipulatorEndEffectorPoseDriverService);
	
	

	endEffectorPoseDriverDefaultLoop.setInterface(this);
	endEffectorPoseDriverDefaultLoop.setTransportInterface(this);
	accessStateMachine.addDefaultStateTransition(endEffectorPoseDriverDefaultLoop);
	
	endEffectorPoseDriverControlledLoop.setInterface(this);
	endEffectorPoseDriverControlledLoop.setTransportInterface(this);
	controlled.addTransition(endEffectorPoseDriverControlledLoop);
	
    
    
	// Start of user code for Constructor:
	// End of user code
}

ManipulatorEndEffectorPoseDriver::~ManipulatorEndEffectorPoseDriver()
{
	// Start of user code for Destructor:
	// End of user code
}

bool ManipulatorEndEffectorPoseDriver::setEndEffectorPose(SetEndEffectorPose *setEndEffectorPose)
{
	// Start of user code for action setEndEffectorPose(SetEndEffectorPose *setEndEffectorPose):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorPoseDriver::setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile)
{
	// Start of user code for action setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorPoseDriver::setToolOffset(SetToolOffset *setToolOffset)
{
	// Start of user code for action setToolOffset(SetToolOffset *setToolOffset):
	return false;
	// End of user code
}

manipulator::ReportManipulatorSpecifications ManipulatorEndEffectorPoseDriver::getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications)
{
	// Start of user code for action getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications):
	manipulator::ReportManipulatorSpecifications message;
	return message;
	// End of user code
}

manipulator::ReportToolOffset ManipulatorEndEffectorPoseDriver::getReportToolOffset(QueryToolOffset *queryToolOffset)
{
	// Start of user code for action getReportToolOffset(QueryToolOffset *queryToolOffset):
	manipulator::ReportToolOffset message;
	return message;
	// End of user code
}

manipulator::ReportCommandedEndEffectorPose ManipulatorEndEffectorPoseDriver::getReportCommandedEndEffectorPose(QueryCommandedEndEffectorPose *queryCommandedEndEffectorPose)
{
	// Start of user code for action getReportCommandedEndEffectorPose(QueryCommandedEndEffectorPose *queryCommandedEndEffectorPose):
	manipulator::ReportCommandedEndEffectorPose message;
	return message;
	// End of user code
}

manipulator::ReportJointMotionProfile ManipulatorEndEffectorPoseDriver::getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile)
{
	// Start of user code for action getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile):
	manipulator::ReportJointMotionProfile message;
	return message;
	// End of user code
}


bool ManipulatorEndEffectorPoseDriver::isControllingEndEffectorPoseClient(SetJointMotionProfile *setJointMotionProfile)
{
	// Start of user code for action isControllingEndEffectorPoseClient(SetJointMotionProfile *setJointMotionProfile):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorPoseDriver::isControllingEndEffectorPoseClient(SetToolOffset *setToolOffset)
{
	// Start of user code for action isControllingEndEffectorPoseClient(SetToolOffset *setToolOffset):
	return false;
	// End of user code
}

bool ManipulatorEndEffectorPoseDriver::isControllingEndEffectorPoseClient(SetEndEffectorPose *setEndEffectorPose)
{
	// Start of user code for action isControllingEndEffectorPoseClient(SetEndEffectorPose *setEndEffectorPose):
	return false;
	// End of user code
}


// Start of user code for additional methods
// End of user code

} // namespace component
} // namespace openjaus

