/**
\file PanTiltJointVelocityDriverDefaultLoop.h

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
#include "openjaus/manipulator/Transitions/PanTiltJointVelocityDriverDefaultLoop.h"
#include "openjaus/manipulator/Triggers/QueryPanTiltSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryCommandedPanTiltJointVelocity.h"
#include "openjaus/manipulator/Triggers/QueryPanTiltMotionProfile.h"
	
namespace openjaus
{
namespace manipulator
{

PanTiltJointVelocityDriverDefaultLoop::PanTiltJointVelocityDriverDefaultLoop()
{
	setName("PanTiltJointVelocityDriverDefaultLoop");
	setType(model::LOOPBACK);
}

PanTiltJointVelocityDriverDefaultLoop::~PanTiltJointVelocityDriverDefaultLoop()
{
}

bool PanTiltJointVelocityDriverDefaultLoop::processTrigger(model::Trigger* trigger)
{
	model::Message *message = dynamic_cast<model::Message *>(trigger);

	switch(trigger->getId())
	{
		case QueryCommandedPanTiltJointVelocity::ID:
		{
			QueryCommandedPanTiltJointVelocity queryCommandedPanTiltJointVelocity(message);
			
			ReportCommandedPanTiltJointVelocity *reportCommandedPanTiltJointVelocity = new ReportCommandedPanTiltJointVelocity();
			*reportCommandedPanTiltJointVelocity = cmptInterface->getReportCommandedPanTiltJointVelocity(&queryCommandedPanTiltJointVelocity);
			reportCommandedPanTiltJointVelocity->setDestination(message->getSource());
			transport->sendMessage(reportCommandedPanTiltJointVelocity);			
			return true;
			break;
		}
			
		case QueryPanTiltSpecifications::ID:
		{
			QueryPanTiltSpecifications queryPanTiltSpecifications(message);
			
			ReportPanTiltSpecifications *reportPanTiltSpecifications = new ReportPanTiltSpecifications();
			*reportPanTiltSpecifications = cmptInterface->getReportPanTiltSpecifications(&queryPanTiltSpecifications);
			reportPanTiltSpecifications->setDestination(message->getSource());
			transport->sendMessage(reportPanTiltSpecifications);			
			return true;
			break;
		}
			
		case QueryPanTiltMotionProfile::ID:
		{
			QueryPanTiltMotionProfile queryPanTiltMotionProfile(message);
			
			ReportPanTiltMotionProfile *reportPanTiltMotionProfile = new ReportPanTiltMotionProfile();
			*reportPanTiltMotionProfile = cmptInterface->getReportPanTiltMotionProfile(&queryPanTiltMotionProfile);
			reportPanTiltMotionProfile->setDestination(message->getSource());
			transport->sendMessage(reportPanTiltMotionProfile);			
			return true;
			break;
		}
			
	}
	
	return false;
}

model::Message* PanTiltJointVelocityDriverDefaultLoop::getResponse(model::Trigger* trigger)
{	
	model::Message *message = dynamic_cast<model::Message *>(trigger);
	
	switch(trigger->getId())
	{
		case QueryCommandedPanTiltJointVelocity::ID:
		{	
			QueryCommandedPanTiltJointVelocity queryCommandedPanTiltJointVelocity(message);
			ReportCommandedPanTiltJointVelocity *reportCommandedPanTiltJointVelocity = new ReportCommandedPanTiltJointVelocity();
			*reportCommandedPanTiltJointVelocity = cmptInterface->getReportCommandedPanTiltJointVelocity(&queryCommandedPanTiltJointVelocity);
			return reportCommandedPanTiltJointVelocity;
		}
			
		case QueryPanTiltSpecifications::ID:
		{	
			QueryPanTiltSpecifications queryPanTiltSpecifications(message);
			ReportPanTiltSpecifications *reportPanTiltSpecifications = new ReportPanTiltSpecifications();
			*reportPanTiltSpecifications = cmptInterface->getReportPanTiltSpecifications(&queryPanTiltSpecifications);
			return reportPanTiltSpecifications;
		}
			
		case QueryPanTiltMotionProfile::ID:
		{	
			QueryPanTiltMotionProfile queryPanTiltMotionProfile(message);
			ReportPanTiltMotionProfile *reportPanTiltMotionProfile = new ReportPanTiltMotionProfile();
			*reportPanTiltMotionProfile = cmptInterface->getReportPanTiltMotionProfile(&queryPanTiltMotionProfile);
			return reportPanTiltMotionProfile;
		}
			
	}
	
	return NULL;
}


bool PanTiltJointVelocityDriverDefaultLoop::setInterface(PanTiltJointVelocityDriverInterface *cmptInterface)
{
	this->cmptInterface = cmptInterface;
	return true;
}

bool PanTiltJointVelocityDriverDefaultLoop::setTransportInterface(core::TransportInterface *cmptInterface)
{
	this->transport = dynamic_cast<core::TransportInterface *>(cmptInterface);
	return true;
}

} // namespace manipulator
} // namespace openjaus

