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

#ifndef GLOBALWAYPOINTLISTDRIVER_SERVICE_INTERFACE_H
#define GLOBALWAYPOINTLISTDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/mobility/ListManagerInterface.h"
#include "openjaus/mobility/Triggers/ExecuteList.h"
#include "openjaus/mobility/Triggers/QueryActiveElement.h"
#include "openjaus/mobility/Triggers/ReportActiveElement.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/QueryGlobalWaypoint.h"
#include "openjaus/mobility/Triggers/ReportGlobalWaypoint.h"
namespace openjaus
{
namespace mobility
{

/// \class GlobalWaypointListDriverInterface GlobalWaypointListDriverInterface.h
/// \brief Provides an abstract interface for the %GlobalWaypointListDriver service. 
/// <p>
/// The function of the Global Waypoint List Driver is to move the platform given a series of target waypoints, desired
/// travel speed, current platform pose and current velocity state. The sequence of waypoints is specified by one or
/// more SetElement messages. A waypoint consists of the desired position and orientation of the platform. The second
/// input consists of the desired travel speed and an optional starting element. The desired travel speed remains
/// unchanged unless a new ExecuteList command is received. The travel speed may then be changed at any time during
/// waypoint navigation. The travel speed is reset to zero for all transitions from the Ready State.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:GlobalWaypointListDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:mobility:ListManager</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT GlobalWaypointListDriverInterface
{
public:
	virtual ~GlobalWaypointListDriverInterface(){};
	
	virtual void resetGwldTravelSpeed() = 0;
	/// \brief SetGlobalWaypointElement action with input SetElement.
	/// SetGlobalWaypointElement action with input SetElement.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setGlobalWaypointElement(SetElement *setElement) = 0;

	/// \brief ExecuteGlobalWaypointList action with input ExecuteList.
	/// ExecuteGlobalWaypointList action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool executeGlobalWaypointList(ExecuteList *executeList) = 0;

	/// \brief ModifyGwldTravelSpeed action with input ExecuteList.
	/// ModifyGwldTravelSpeed action with input ExecuteList.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool modifyGwldTravelSpeed(ExecuteList *executeList) = 0;

	/// \brief Send action for ReportGlobalWaypoint with input message QueryGlobalWaypoint.
	/// Send action for ReportGlobalWaypoint with input message QueryGlobalWaypoint.
	/// \param[in] queryGlobalWaypoint - Input Trigger.
	/// \return ReportGlobalWaypoint Output Message.
	virtual ReportGlobalWaypoint getReportGlobalWaypoint(QueryGlobalWaypoint *queryGlobalWaypoint) = 0;

	/// \brief Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// Send action for ReportTravelSpeed with input message QueryTravelSpeed.
	/// \param[in] queryTravelSpeed - Input Trigger.
	/// \return ReportTravelSpeed Output Message.
	virtual ReportTravelSpeed getReportTravelSpeed(QueryTravelSpeed *queryTravelSpeed) = 0;

	/// \brief Send action for ReportActiveElement with input message QueryActiveElement.
	/// Send action for ReportActiveElement with input message QueryActiveElement.
	/// \param[in] queryActiveElement - Input Trigger.
	/// \return ReportActiveElement Output Message.
	virtual ReportActiveElement getReportActiveElement(QueryActiveElement *queryActiveElement) = 0;

	/// \brief Send action for ConfirmElementRequest with input message SetElement.
	/// Send action for ConfirmElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return ConfirmElementRequest Output Message.
	virtual ConfirmElementRequest getConfirmElementRequest(SetElement *setElement) = 0;

	/// \brief Send action for RejectElementRequest with input message SetElement.
	/// Send action for RejectElementRequest with input message SetElement.
	/// \param[in] setElement - Input Trigger.
	/// \return RejectElementRequest Output Message.
	virtual RejectElementRequest getRejectElementRequest(SetElement *setElement) = 0;

	/// \brief isControllingGwldClient condition.
	/// isControllingGwldClient condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGwldClient(SetElement *setElement) = 0;

	/// \brief isControllingGwldClient condition.
	/// isControllingGwldClient condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGwldClient(ExecuteList *executeList) = 0;

	/// \brief gwldWaypointExists condition.
	/// gwldWaypointExists condition.
	/// \param[in]  queryGlobalWaypoint - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldWaypointExists(QueryGlobalWaypoint *queryGlobalWaypoint) = 0;

	/// \brief gwldWaypointExists condition.
	/// gwldWaypointExists condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldWaypointExists(ExecuteList *executeList) = 0;

	/// \brief gwldElementExists condition.
	/// gwldElementExists condition.
	/// \param[in] trigger - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldElementExists(model::Trigger *trigger) = 0;

	/// \brief isValidGwldElementRequest condition.
	/// isValidGwldElementRequest condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isValidGwldElementRequest(SetElement *setElement) = 0;

	/// \brief isGwldElementSupported condition.
	/// isGwldElementSupported condition.
	/// \param[in]  setElement - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isGwldElementSupported(SetElement *setElement) = 0;

	/// \brief gwldElementSpecified condition.
	/// gwldElementSpecified condition.
	/// \param[in]  executeList - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool gwldElementSpecified(ExecuteList *executeList) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // GLOBALWAYPOINTLISTDRIVER_SERVICE_INTERFACE_H
