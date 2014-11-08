/**
\file ManipulatorEndEffectorPoseListDriver.h

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

#ifndef MANIPULATORENDEFFECTORPOSELISTDRIVER_COMPONENT_H
#define MANIPULATORENDEFFECTORPOSELISTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/manipulator/ManipulatorEndEffectorPoseListDriverInterface.h"
#include "openjaus/manipulator/Transitions/EndEffectorPoseListDriverDefaultLoop.h"
#include "openjaus/manipulator/Transitions/EndEffectorPoseListDriverControlledLoop.h"
#include "openjaus/manipulator/Triggers/ExecuteList.h"
#include "openjaus/manipulator/Triggers/SetJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/SetToolOffset.h"
#include "openjaus/manipulator/Triggers/QueryToolOffset.h"
#include "openjaus/manipulator/Triggers/QueryActiveElement.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/QueryCommandedEndEffectorPose.h"
#include "openjaus/manipulator/Triggers/ReportActiveElement.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/ReportCommandedEndEffectorPose.h"
#include "openjaus/manipulator/Triggers/ReportToolOffset.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorEndEffectorPoseListDriver ManipulatorEndEffectorPoseListDriver.h
/// \brief %ManipulatorEndEffectorPoseListDriver Component implements the urn:jaus:jss:manipulator:ManipulatorEndEffectorPoseListDriver services.
/// The %ManipulatorEndEffectorPoseListDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%ManipulatorEndEffectorPoseListDriver Service</dt>
/// <dd><p>
/// The function of the End-Effector Pose List Driver is to perform closed-loop control of a sequence of positions and
/// orientations of the tool tip specified in vehicle coordinate system. The sequence of targets is specified by one or
/// more SetElement messages, as defined by the List Manager Service. The "Set Motion Profile" message is used to set
/// maximum velocity and acceleration rates for each of the variable joint parameters.  All motions utilize the motion
/// profile data that was most recently sent. Default settings are not assumed so that upon initialization this message
/// must be sent before the first "ExecuteList" message is sent.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:ManipulatorEndEffectorPoseListDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT ManipulatorEndEffectorPoseListDriver : public core::Managed, public manipulator::ManipulatorEndEffectorPoseListDriverInterface
{

public:
	ManipulatorEndEffectorPoseListDriver();
	virtual ~ManipulatorEndEffectorPoseListDriver();

	/// \brief Begin sequential execution of the target list starting at the specified element.
	/// Begin sequential execution of the target list starting at the specified element.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool executeTargetList(ExecuteList *executeList);

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

	/// \brief Send a Report Motion Profile message
	/// Send a Report Motion Profile message
	/// \param[in] queryJointMotionProfile - Input Trigger.
	/// \return ReportJointMotionProfile Output Message.
	virtual ReportJointMotionProfile getReportJointMotionProfile(QueryJointMotionProfile *queryJointMotionProfile);

	/// \brief Send a Report Active Element message
	/// Send a Report Active Element message
	/// \param[in] queryActiveElement - Input Trigger.
	/// \return ReportActiveElement Output Message.
	virtual ReportActiveElement getReportActiveElement(QueryActiveElement *queryActiveElement);

	/// \brief Send a Report Commanded End Effector Pose message
	/// Send a Report Commanded End Effector Pose message
	/// \param[in] queryCommandedEndEffectorPose - Input Trigger.
	/// \return ReportCommandedEndEffectorPose Output Message.
	virtual ReportCommandedEndEffectorPose getReportCommandedEndEffectorPose(QueryCommandedEndEffectorPose *queryCommandedEndEffectorPose);

	/// \brief Send a Report Tool Offset message
	/// Send a Report Tool Offset message
	/// \param[in] queryToolOffset - Input Trigger.
	/// \return ReportToolOffset Output Message.
	virtual ReportToolOffset getReportToolOffset(QueryToolOffset *queryToolOffset);


	/// \brief isControllingEndEffectorPoseListClient condition.
	/// isControllingEndEffectorPoseListClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorPoseListClient(ExecuteList *executeList);

	/// \brief isControllingEndEffectorPoseListClient condition.
	/// isControllingEndEffectorPoseListClient condition.
	/// \param[in]  setJointMotionProfile - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorPoseListClient(SetJointMotionProfile *setJointMotionProfile);

	/// \brief isControllingEndEffectorPoseListClient condition.
	/// isControllingEndEffectorPoseListClient condition.
	/// \param[in]  setToolOffset - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingEndEffectorPoseListClient(SetToolOffset *setToolOffset);


	// Start of user code for additional methods:
	// End of user code

protected:
	EndEffectorPoseListDriverDefaultLoop endEffectorPoseListDriverDefaultLoop;
	EndEffectorPoseListDriverControlledLoop endEffectorPoseListDriverControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // MANIPULATORENDEFFECTORPOSELISTDRIVER_H
