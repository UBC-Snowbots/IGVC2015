/**
\file EndEffectorPoseDriverDefaultLoop.h

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

#include <openjaus.h>
#include "openjaus/manipulator/Transitions/EndEffectorPoseDriverDefaultLoop.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryToolOffset.h"
#include "openjaus/manipulator/Triggers/QueryJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/QueryCommandedEndEffectorVelocityState.h"
	
namespace openjaus
{
namespace manipulator
{

EndEffectorPoseDriverDefaultLoop::EndEffectorPoseDriverDefaultLoop()
{
	setName("EndEffectorPoseDriverDefaultLoop");
	setType(model::LOOPBACK);
}

EndEffectorPoseDriverDefaultLoop::~EndEffectorPoseDriverDefaultLoop()
{
}

bool EndEffectorPoseDriverDefaultLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryManipulatorSpecifications::ID:
		{
			QueryManipulatorSpecifications queryManipulatorSpecifications(message);
			
			ReportManipulatorSpecifications *reportManipulatorSpecifications = new ReportManipulatorSpecifications();
			*reportManipulatorSpecifications = cmptInterface->getReportManipulatorSpecifications(&queryManipulatorSpecifications);
			reportManipulatorSpecifications->setDestination(message->getSource());
			transport->sendMessage(reportManipulatorSpecifications);			
			return true;
			break;
		}
			
		case QueryToolOffset::ID:
		{
			QueryToolOffset queryToolOffset(message);
			
			ReportToolOffset *reportToolOffset = new ReportToolOffset();
			*reportToolOffset = cmptInterface->getReportToolOffset(&queryToolOffset);
			reportToolOffset->setDestination(message->getSource());
			transport->sendMessage(reportToolOffset);			
			return true;
			break;
		}
			
		case QueryJointMotionProfile::ID:
		{
			QueryJointMotionProfile queryJointMotionProfile(message);
			
			ReportJointMotionProfile *reportJointMotionProfile = new ReportJointMotionProfile();
			*reportJointMotionProfile = cmptInterface->getReportJointMotionProfile(&queryJointMotionProfile);
			reportJointMotionProfile->setDestination(message->getSource());
			transport->sendMessage(reportJointMotionProfile);			
			return true;
			break;
		}
			
		case QueryCommandedEndEffectorPose::ID:
		{
			QueryCommandedEndEffectorPose queryCommandedEndEffectorPose(message);
			
			ReportCommandedEndEffectorPose *reportCommandedEndEffectorPose = new ReportCommandedEndEffectorPose();
			*reportCommandedEndEffectorPose = cmptInterface->getReportCommandedEndEffectorPose(&queryCommandedEndEffectorPose);
			reportCommandedEndEffectorPose->setDestination(message->getSource());
			transport->sendMessage(reportCommandedEndEffectorPose);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* EndEffectorPoseDriverDefaultLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryManipulatorSpecifications::ID:
		{	
			QueryManipulatorSpecifications queryManipulatorSpecifications(message);
			ReportManipulatorSpecifications *reportManipulatorSpecifications = new ReportManipulatorSpecifications();
			*reportManipulatorSpecifications = cmptInterface->getReportManipulatorSpecifications(&queryManipulatorSpecifications);
			return reportManipulatorSpecifications;
		}
			
		case QueryToolOffset::ID:
		{	
			QueryToolOffset queryToolOffset(message);
			ReportToolOffset *reportToolOffset = new ReportToolOffset();
			*reportToolOffset = cmptInterface->getReportToolOffset(&queryToolOffset);
			return reportToolOffset;
		}
			
		case QueryJointMotionProfile::ID:
		{	
			QueryJointMotionProfile queryJointMotionProfile(message);
			ReportJointMotionProfile *reportJointMotionProfile = new ReportJointMotionProfile();
			*reportJointMotionProfile = cmptInterface->getReportJointMotionProfile(&queryJointMotionProfile);
			return reportJointMotionProfile;
		}
			
		case QueryCommandedEndEffectorPose::ID:
		{	
			QueryCommandedEndEffectorPose queryCommandedEndEffectorPose(message);
			ReportCommandedEndEffectorPose *reportCommandedEndEffectorPose = new ReportCommandedEndEffectorPose();
			*reportCommandedEndEffectorPose = cmptInterface->getReportCommandedEndEffectorPose(&queryCommandedEndEffectorPose);
			return reportCommandedEndEffectorPose;
		}
			
	}
	
	return NULL;
}


bool EndEffectorPoseDriverDefaultLoop::setInterface(ManipulatorEndEffectorPoseDriverInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool EndEffectorPoseDriverDefaultLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace manipulator
} // namespace openjaus

