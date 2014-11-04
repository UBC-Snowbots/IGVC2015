/**
\file ManipulatorJointVelocityDriver.h

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

#ifndef MANIPULATORJOINTVELOCITYDRIVER_SERVICE_INTERFACE_H
#define MANIPULATORJOINTVELOCITYDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/SetJointVelocity.h"
#include "openjaus/manipulator/Triggers/SetJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/QueryCommandedJointVelocity.h"
#include "openjaus/manipulator/Triggers/QueryJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportCommandedJointVelocity.h"
#include "openjaus/manipulator/Triggers/ReportJointMotionProfile.h"
namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorJointVelocityDriverInterface ManipulatorJointVelocityDriverInterface.h
/// \brief Provides an abstract interface for the %ManipulatorJointVelocityDriver service. 
/// <p>
/// The function of the Joint Velocity Driver is to perform closed-loop joint velocity control.  The input is the
/// desired instantaneous desired joint velocities.  The "Set Motion Profile" message is used to set maximum velocity
/// and acceleration rates for each of the variable joint parameters.  All motions utilize the motion profile data that
/// was most recently sent.  Default settings are not assumed so that upon initialization this message must be sent
/// before the first "SET JOINT VELOCITY" message is sent.  It is assumed that the manipulator begins motion immediately
/// after receiving the "SET JOINT VELOCITY" message.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:manipulator:ManipulatorJointVelocityDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT ManipulatorJointVelocityDriverInterface
{
public:
	virtual ~ManipulatorJointVelocityDriverInterface(){};
	
	/// \brief Set the desired velocities for the individual joints of the manipulator
	/// Set the desired velocities for the individual joints of the manipulator
	/// \param[in]  setJointVelocity - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setJointVelocity(SetJointVelocity *setJointVelocity) = 0;

	/// \brief Set the joint motion profile parameters for the manipulator.
	/// Set the joint motion profile parameters for the manipulator.
	/// \param[in]  setJointMotionProfile - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile) = 0;

	/// \brief Send a report manipulator specs message
	/// Send a report manipulator specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications) = 0;

	/// \brief Send a Report Commanded joint velocity message
	/// Send a Report Commanded joint velocity message
	/// \param[in] queryCommandedJointVelocity - Input Trigger.
	/// \return ReportCommandedJointVelocity Output Message.
	virtual ReportCommandedJointVelocity getReportCommandedJointVelocity(QueryCommandedJointVelocity *queryCommandedJointVelocity) = 0;

	/// \brief Send a Report Motion Profile message
	/// Send a Report Motion Profile message
	/// \param[in] queryJointMotionProfile - Input Trigger.
	/// \return ReportJointMotionProfile Output Message.
	virtual ReportJointMotionProfile getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile) = 0;

	/// \brief isControllingJointVelocityClient condition.
	/// isControllingJointVelocityClient condition.
	/// \param[in]  setJointMotionProfile - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingJointVelocityClient(SetJointMotionProfile *setJointMotionProfile) = 0;

	/// \brief isControllingJointVelocityClient condition.
	/// isControllingJointVelocityClient condition.
	/// \param[in]  setJointVelocity - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingJointVelocityClient(SetJointVelocity *setJointVelocity) = 0;

};

} // namespace manipulator
} // namespace openjaus

#endif // MANIPULATORJOINTVELOCITYDRIVER_SERVICE_INTERFACE_H
