/**
\file ManipulatorJointPositionListDriver.h

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

#ifndef MANIPULATORJOINTPOSITIONLISTDRIVER_COMPONENT_H
#define MANIPULATORJOINTPOSITIONLISTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/manipulator/ManipulatorJointPositionListDriverInterface.h"
#include "openjaus/manipulator/Transitions/JointPositionListDriverDefaultLoop.h"
#include "openjaus/manipulator/Transitions/JointPositionListDriverControlledLoop.h"
#include "openjaus/manipulator/Triggers/ExecuteList.h"
#include "openjaus/manipulator/Triggers/SetJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/QueryActiveElement.h"
#include "openjaus/manipulator/Triggers/QueryManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/QueryJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/QueryCommandedJointPosition.h"
#include "openjaus/manipulator/Triggers/ReportActiveElement.h"
#include "openjaus/manipulator/Triggers/ReportManipulatorSpecifications.h"
#include "openjaus/manipulator/Triggers/ReportJointMotionProfile.h"
#include "openjaus/manipulator/Triggers/ReportCommandedJointPosition.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace manipulator
{

/// \class ManipulatorJointPositionListDriver ManipulatorJointPositionListDriver.h
/// \brief %ManipulatorJointPositionListDriver Component implements the urn:jaus:jss:manipulator:ManipulatorJointPositionListDriver services.
/// The %ManipulatorJointPositionListDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%ManipulatorJointPositionListDriver Service</dt>
/// <dd><p>
/// The function of the Joint Position List Driver is to perform closed-loop joint position control through a sequence
/// of targets. The sequence of targets is specified by one or more SetElement messages, as defined by the List Manager
/// Service. The "Set Motion Profile" message is used to set maximum velocity and acceleration rates for each of the
/// variable joint parameters.  All motions utilize the motion profile data that was most recently sent.  Default
/// settings are not assumed so that upon initialization this message must be sent before the first "ExecuteList"
/// message is sent.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:manipulator:ManipulatorJointPositionListDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT ManipulatorJointPositionListDriver : public core::Managed, public manipulator::ManipulatorJointPositionListDriverInterface
{

public:
	ManipulatorJointPositionListDriver();
	virtual ~ManipulatorJointPositionListDriver();

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

	/// Store the given targets(s) in the target list with sequence specified by the previous and next element UIDs.  If this action represents an insert or append into an existing list, the service should modify the NextUID of the previous element and/or the PreviousUID of the next element to reflect the updated sequence
    virtual bool setElement(model::Trigger *trigger);

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

	/// \brief Send a Report Commanded Joint Position message
	/// Send a Report Commanded Joint Position message
	/// \param[in] queryCommandedJointPosition - Input Trigger.
	/// \return ReportCommandedJointPosition Output Message.
	virtual ReportCommandedJointPosition getReportCommandedJointPosition(QueryCommandedJointPosition *queryCommandedJointPosition);


	/// \brief isControllingJointPositionListClient condition.
	/// isControllingJointPositionListClient condition.
	/// \param[in]  setJointMotionProfile - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingJointPositionListClient(SetJointMotionProfile *setJointMotionProfile);

	/// \brief isControllingJointPositionListClient condition.
	/// isControllingJointPositionListClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingJointPositionListClient(ExecuteList *executeList);


	// Start of user code for additional methods:
	// End of user code

protected:
	JointPositionListDriverDefaultLoop jointPositionListDriverDefaultLoop;
	JointPositionListDriverControlledLoop jointPositionListDriverControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // MANIPULATORJOINTPOSITIONLISTDRIVER_H
