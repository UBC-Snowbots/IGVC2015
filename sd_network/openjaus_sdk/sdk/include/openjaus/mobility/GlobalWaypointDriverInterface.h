/**
\file GlobalWaypointDriver.h

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

#ifndef GLOBALWAYPOINTDRIVER_SERVICE_INTERFACE_H
#define GLOBALWAYPOINTDRIVER_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/ManagementInterface.h"
#include "openjaus/mobility/Triggers/SetTravelSpeed.h"
#include "openjaus/mobility/Triggers/SetGlobalWaypoint.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/QueryGlobalWaypoint.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportGlobalWaypoint.h"
namespace openjaus
{
namespace mobility
{

/// \class GlobalWaypointDriverInterface GlobalWaypointDriverInterface.h
/// \brief Provides an abstract interface for the %GlobalWaypointDriver service. 
/// <p>
/// The function of the Global Waypoint Driver is to move the platform given a single target waypoint, desired travel
/// speed, current platform pose and current velocity state. A single waypoint is provided via the Set Global Waypoint
/// message. The waypoint remains unchanged until a new Set Global Waypoint message is received. A waypoint consists of
/// the desired position and orientation of the platform. The second input consists of the desired travel speed. The
/// desired travel speed remains unchanged unless a new Set Travel Speed Message is received. The travel speed may then
/// be changed at any time during waypoint navigation. The travel speed is reset to zero for all transitions from the
/// Ready State.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:mobility:GlobalWaypointDriver<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Management</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT GlobalWaypointDriverInterface
{
public:
	virtual ~GlobalWaypointDriverInterface(){};
	
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

	/// \brief SetGwdTravelSpeed action with input SetTravelSpeed.
	/// SetGwdTravelSpeed action with input SetTravelSpeed.
	/// \param[in]  setTravelSpeed - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setGwdTravelSpeed(SetTravelSpeed *setTravelSpeed) = 0;

	/// \brief SetGlobalWaypoint action with input SetGlobalWaypoint.
	/// SetGlobalWaypoint action with input SetGlobalWaypoint.
	/// \param[in]  setGlobalWaypoint - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool setGlobalWaypoint(SetGlobalWaypoint *setGlobalWaypoint) = 0;

	virtual void resetGwdTravelSpeed() = 0;
	/// \brief True if a valide waypoint has been received.
	/// True if a valide waypoint has been received.
	/// \param[in]  queryGlobalWaypoint - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool waypointExists(QueryGlobalWaypoint *queryGlobalWaypoint) = 0;

	/// \brief True if a valide waypoint has been received.
	/// True if a valide waypoint has been received.
	/// \param[in]  setTravelSpeed - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool waypointExists(SetTravelSpeed *setTravelSpeed) = 0;

	/// \brief True if a valide waypoint has been received.
	/// True if a valide waypoint has been received.
	/// \param[in]  setGlobalWaypoint - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool waypointExists(SetGlobalWaypoint *setGlobalWaypoint) = 0;

	/// \brief isControllingGwdClient condition.
	/// isControllingGwdClient condition.
	/// \param[in]  setTravelSpeed - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingGwdClient(SetTravelSpeed *setTravelSpeed) = 0;

};

} // namespace mobility
} // namespace openjaus

#endif // GLOBALWAYPOINTDRIVER_SERVICE_INTERFACE_H
