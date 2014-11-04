/**
\file GlobalWaypointListDriver.h

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

#ifndef GLOBALWAYPOINTLISTDRIVER_COMPONENT_H
#define GLOBALWAYPOINTLISTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/mobility/ListManaged.h"
#include "openjaus/mobility/GlobalWaypointListDriverInterface.h"
#include "openjaus/mobility/Transitions/GwldDefaultLoop.h"
#include "openjaus/mobility/Transitions/GwldControlledLoop.h"
#include "openjaus/mobility/Transitions/GwldReadyLoop.h"
#include "openjaus/mobility/Triggers/ExecuteList.h"
#include "openjaus/mobility/Triggers/QueryActiveElement.h"
#include "openjaus/mobility/Triggers/ReportActiveElement.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/QueryGlobalWaypoint.h"
#include "openjaus/mobility/Triggers/ReportGlobalWaypoint.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class GlobalWaypointListDriver GlobalWaypointListDriver.h
/// \brief %GlobalWaypointListDriver Component implements the urn:jaus:jss:mobility:GlobalWaypointListDriver services.
/// The %GlobalWaypointListDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%GlobalWaypointListDriver Service</dt>
/// <dd><p>
/// The function of the Global Waypoint List Driver is to move the platform given a series of target waypoints, desired
/// travel speed, current platform pose and current velocity state. The sequence of waypoints is specified by one or
/// more SetElement messages. A waypoint consists of the desired position and orientation of the platform. The second
/// input consists of the desired travel speed and an optional starting element. The desired travel speed remains
/// unchanged unless a new ExecuteList command is received. The travel speed may then be changed at any time during
/// waypoint navigation. The travel speed is reset to zero for all transitions from the Ready State.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:GlobalWaypointListDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:mobility:ListManager</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT GlobalWaypointListDriver : public mobility::ListManaged, public mobility::GlobalWaypointListDriverInterface
{

public:
	GlobalWaypointListDriver();
	virtual ~GlobalWaypointListDriver();

	virtual void resetGwldTravelSpeed();

	/// \brief SetGlobalWaypointElement action with input SetElement.
	/// SetGlobalWaypointElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setGlobalWaypointElement(SetElement *setElement);

	/// \brief ExecuteGlobalWaypointList action with input ExecuteList.
	/// ExecuteGlobalWaypointList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool executeGlobalWaypointList(ExecuteList *executeList);

	/// \brief ModifyGwldTravelSpeed action with input ExecuteList.
	/// ModifyGwldTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool modifyGwldTravelSpeed(ExecuteList *executeList);

	/// \brief Send action for ReportGlobalWaypoint with input message QueryGlobalWaypoint.
	/// Send action for ReportGlobalWaypoint with input message QueryGlobalWaypoint.
	/// \param[in] queryGlobalWaypoint - Input Trigger.
	/// \return ReportGlobalWaypoint Output Message.
	virtual ReportGlobalWaypoint getReportGlobalWaypoint(QueryGlobalWaypoint *queryGlobalWaypoint);

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


	/// \brief isControllingGwldClient condition.
	/// isControllingGwldClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGwldClient(SetElement *setElement);

	/// \brief isControllingGwldClient condition.
	/// isControllingGwldClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGwldClient(ExecuteList *executeList);


	/// \brief gwldWaypointExists condition.
	/// gwldWaypointExists condition.
	/// \param[in]  queryGlobalWaypoint - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldWaypointExists(QueryGlobalWaypoint *queryGlobalWaypoint);

	/// \brief gwldWaypointExists condition.
	/// gwldWaypointExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldWaypointExists(ExecuteList *executeList);


	/// \brief gwldElementExists condition.
	/// gwldElementExists condition.
	/// \param[in]  trigger - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldElementExists(model::Trigger *trigger);


	/// \brief isValidGwldElementRequest condition.
	/// isValidGwldElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidGwldElementRequest(SetElement *setElement);


	/// \brief isGwldElementSupported condition.
	/// isGwldElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isGwldElementSupported(SetElement *setElement);


	/// \brief gwldElementSpecified condition.
	/// gwldElementSpecified condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldElementSpecified(ExecuteList *executeList);


	// Start of user code for additional methods:
	// End of user code

protected:
	GwldDefaultLoop gwldDefaultLoop;
	GwldControlledLoop gwldControlledLoop;
	GwldReadyLoop gwldReadyLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // GLOBALWAYPOINTLISTDRIVER_H
