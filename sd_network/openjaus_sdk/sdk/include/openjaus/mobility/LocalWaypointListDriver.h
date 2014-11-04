/**
\file LocalWaypointListDriver.h

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

#ifndef LOCALWAYPOINTLISTDRIVER_COMPONENT_H
#define LOCALWAYPOINTLISTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/mobility/ListManaged.h"
#include "openjaus/mobility/LocalWaypointListDriverInterface.h"
#include "openjaus/mobility/Transitions/LwldDefaultLoop.h"
#include "openjaus/mobility/Transitions/LwldControlledLoop.h"
#include "openjaus/mobility/Transitions/LwldReadyLoop.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/QueryLocalWaypoint.h"
#include "openjaus/mobility/Triggers/ReportLocalWaypoint.h"
#include "openjaus/mobility/Triggers/ExecuteList.h"
#include "openjaus/mobility/Triggers/QueryActiveElement.h"
#include "openjaus/mobility/Triggers/ReportActiveElement.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class LocalWaypointListDriver LocalWaypointListDriver.h
/// \brief %LocalWaypointListDriver Component implements the urn:jaus:jss:mobility:LocalWaypointListDriver services.
/// The %LocalWaypointListDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%LocalWaypointListDriver Service</dt>
/// <dd><p>
/// The function of the Local Waypoint List Driver is to move the platform given a series of target waypoints, desired
/// travel speed, current platform pose and current velocity state. The sequence of waypoints is specified by one or
/// more SetElement messages. A waypoint consists of the desired position and orientation of the platform. The second
/// input consists of the desired travel speed and an optional starting element. The desired travel speed remains
/// unchanged unless a new ExecuteList command is received. The travel speed may then be changed at any time during
/// waypoint navigation. The travel speed is reset to zero for all transitions from the Ready State.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:LocalWaypointListDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:mobility:ListManager</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT LocalWaypointListDriver : public mobility::ListManaged, public mobility::LocalWaypointListDriverInterface
{

public:
	LocalWaypointListDriver();
	virtual ~LocalWaypointListDriver();

	virtual void resetLwldTravelSpeed();

	/// \brief SetLocalWaypointElement action with input SetElement.
	/// SetLocalWaypointElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setLocalWaypointElement(SetElement *setElement);

	/// \brief ExecuteLocalWaypointList action with input ExecuteList.
	/// ExecuteLocalWaypointList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool executeLocalWaypointList(ExecuteList *executeList);

	/// \brief ModifyLwldTravelSpeed action with input ExecuteList.
	/// ModifyLwldTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool modifyLwldTravelSpeed(ExecuteList *executeList);

	/// \brief Send action for ReportLocalWaypoint with input message QueryLocalWaypoint.
	/// Send action for ReportLocalWaypoint with input message QueryLocalWaypoint.
	/// \param[in] queryLocalWaypoint - Input Trigger.
	/// \return ReportLocalWaypoint Output Message.
	virtual ReportLocalWaypoint getReportLocalWaypoint(QueryLocalWaypoint *queryLocalWaypoint);

	/// \brief Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// \param[in] queryTravelSpeed - Input Trigger.
	/// \return ReportTravelSpeed Output Message.
	virtual ReportTravelSpeed getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed);

	/// \brief Send action for ReportActiveElement with input message QueryActiveElement.
	/// Send action for ReportActiveElement with input message QueryActiveElement.
	/// \param[in] queryActiveElement - Input Trigger.
	/// \return ReportActiveElement Output Message.
	virtual ReportActiveElement getReportActiveElement(QueryActiveElement *queryActiveElement);

	/// \brief Send action for ConfirmElementRequest with input message SetElement.
	/// Send action for ConfirmElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(SetElement *setElement);

	/// \brief Send action for RejectElementRequest with input message SetElement.
	/// Send action for RejectElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(SetElement *setElement);


	/// \brief isControllingLwldClient condition.
	/// isControllingLwldClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLwldClient(SetElement *setElement);

	/// \brief isControllingLwldClient condition.
	/// isControllingLwldClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLwldClient(ExecuteList *executeList);


	/// \brief lwldWaypointExists condition.
	/// lwldWaypointExists condition.
	/// \param[in]  queryLocalWaypoint - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lwldWaypointExists(QueryLocalWaypoint *queryLocalWaypoint);


	/// \brief lwldElementExists condition.
	/// lwldElementExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lwldElementExists(ExecuteList *executeList);


	/// \brief isValidLwldElementRequest condition.
	/// isValidLwldElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidLwldElementRequest(SetElement *setElement);


	/// \brief isLwldElementSupported condition.
	/// isLwldElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isLwldElementSupported(SetElement *setElement);


	/// \brief lwldElementSpecified condition.
	/// lwldElementSpecified condition.
	/// \param[in]  trigger - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool lwldElementSpecified(model::Trigger *trigger);


	// Start of user code for additional methods:
	// End of user code

protected:
	LwldDefaultLoop lwldDefaultLoop;
	LwldControlledLoop lwldControlledLoop;
	LwldReadyLoop lwldReadyLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // LOCALWAYPOINTLISTDRIVER_H
