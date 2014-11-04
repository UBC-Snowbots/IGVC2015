/**
\file ManipulatorEndEffectorVelocityStateDriver.h

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

#ifndef MANIPULATORENDEFFECTORVELOCITYSTATEDRIVER_COMPONENT_H
#define MANIPULATORENDEFFECTORVELOCITYSTATEDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/manipulator/ManipulatorEndEffectorVelocityStateDriverInterface.h"
#include "openjaus/manipulator/Transitions/EndEffectorVelocityStateDriverDefaultLoop.h"
#include "openjaus/manipulator/Transitions/EndEffectorVelocityStateDriverControlledLoop.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryToolOffset.h"
#include "openjaus/manipulator/Triggers/SetToolOffset.h"
#include "openjaus/manipulator/Triggers/SetEndEffectorVelocityState.h"
#include "openjaus/manipulator/Triggers/SetJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/QueryCommandedEndEffectorVelocityState.h"
#include "openjaus/manipulator/Triggers/QueryJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportToolOffset.h"
#include "openjaus/manipulator/Triggers/ReportCommandedEndEffectorVelocityState.h"
#include "openjaus/manipulator/Triggers/ReportJointMotionProfile.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorEndEffectorVelocityStateDriver ManipulatorEndEffectorVelocityStateDriver.h
/// \brief %ManipulatorEndEffectorVelocityStateDriver Component implements the urn:jaus:jss:manipulator:ManipulatorEndEffectorVelocityStateDriver services.
/// The %ManipulatorEndEffectorVelocityStateDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%ManipulatorEndEffectorVelocityStateDriver Service</dt>
/// <dd><p>
/// The function of the End Effector Velocity State Driver is to perform closed-loop velocity control of the tool tip. 
/// The velocity state of the tool tip is defined by two length-three vectors, i.e. ωe and vtool,e.  These vectors
/// respectively represent the angular velocity of the end-effector coordinate system and the linear velocity of the
/// tool tip as measured with respect to the manipulator base system.  The "Set Motion Profile" message is used to set
/// maximum velocity and acceleration rates for each of the variable joint parameters.  All motions utilize the motion
/// profile data that was most recently sent.  Default settings are not assumed so that upon initialization this message
/// must be sent before the first "Set End Effector Velocity State" message is sent.  It is assumed that the manipulator
/// begins motion immediately after receiving the "Set End Effector Velocity State" message.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:ManipulatorEndEffectorVelocityStateDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT ManipulatorEndEffectorVelocityStateDriver : public core::Managed, public manipulator::ManipulatorEndEffectorVelocityStateDriverInterface
{

public:
	ManipulatorEndEffectorVelocityStateDriver();
	virtual ~ManipulatorEndEffectorVelocityStateDriver();

	/// \brief Set the desired velocity state for the end-effector
	/// Set the desired velocity state for the end-effector
	/// \param[in]  setEndEffectorVelocityState - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setEndEffectorVelocityState(SetEndEffectorVelocityState *setEndEffectorVelocityState);

	/// \brief Set the joint motion profile parameters for the manipulator.
	/// Set the joint motion profile parameters for the manipulator.
	/// \param[in]  setJointMotionProfile - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setJointMotionProfile(SetJointMotionProfile *setJointMotionProfile);

	/// \brief Set the location of the tool tip as measured in the end effector coordinate system
	/// Set the location of the tool tip as measured in the end effector coordinate system
	/// \param[in]  setToolOffset - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setToolOffset(SetToolOffset *setToolOffset);

	/// \brief Send a report manipulator specs message
	/// Send a report manipulator specs message
	/// \param[in] queryManipulatorSpecifications - Input Trigger.
	/// \return ReportManipulatorSpecifications Output Message.
	virtual ReportManipulatorSpecifications getReportManipulatorSpecifications(QueryManipulatorSpecifications *queryManipulatorSpecifications);

	/// \brief Send a Report Tool Offset message
	/// Send a Report Tool Offset message
	/// \param[in] queryToolOffset - Input Trigger.
	/// \return ReportToolOffset Output Message.
	virtual ReportToolOffset getReportToolOffset(QueryToolOffset *queryToolOffset);

	/// \brief Send a Report Commanded End effector velocity state message
	/// Send a Report Commanded End effector velocity state message
	/// \param[in] queryCommandedEndEffectorVelocityState - Input Trigger.
	/// \return ReportCommandedEndEffectorVelocityState Output Message.
	virtual ReportCommandedEndEffectorVelocityState getReportCommandedEndEffectorVelocityState(QueryCommandedEndEffectorVelocityState *queryCommandedEndEffectorVelocityState);

	/// \brief Send a Report Motion Profile message
	/// Send a Report Motion Profile message
	/// \param[in] queryJointMotionProfile - Input Trigger.
	/// \return ReportJointMotionProfile Output Message.
	virtual ReportJointMotionProfile getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile);


	/// \brief isControllingEndEffectorVelocityStateClient condition.
	/// isControllingEndEffectorVelocityStateClient condition.
	/// \param[in]  setJointMotionProfile - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorVelocityStateClient(SetJointMotionProfile *setJointMotionProfile);

	/// \brief isControllingEndEffectorVelocityStateClient condition.
	/// isControllingEndEffectorVelocityStateClient condition.
	/// \param[in]  setToolOffset - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorVelocityStateClient(SetToolOffset *setToolOffset);

	/// \brief isControllingEndEffectorVelocityStateClient condition.
	/// isControllingEndEffectorVelocityStateClient condition.
	/// \param[in]  setEndEffectorVelocityState - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorVelocityStateClient(SetEndEffectorVelocityState *setEndEffectorVelocityState);


	// Start of user code for additional methods:
	// End of user code

protected:
	EndEffectorVelocityStateDriverDefaultLoop endEffectorVelocityStateDriverDefaultLoop;
	EndEffectorVelocityStateDriverControlledLoop endEffectorVelocityStateDriverControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // MANIPULATORENDEFFECTORVELOCITYSTATEDRIVER_H
