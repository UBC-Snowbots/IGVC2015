/**
\file LocalWaypointDriver.h

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

#ifndef LOCALWAYPOINTDRIVER_COMPONENT_H
#define LOCALWAYPOINTDRIVER_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Managed.h"
#include "openjaus/mobility/LocalWaypointDriverInterface.h"
#include "openjaus/mobility/Transitions/LwdDefaultLoop.h"
#include "openjaus/mobility/Transitions/LwdReadyLoop.h"
#include "openjaus/mobility/Transitions/LwdControlledLoop.h"
#include "openjaus/mobility/Triggers/SetLocalWaypoint.h"
#include "openjaus/mobility/Triggers/QueryLocalWaypoint.h"
#include "openjaus/mobility/Triggers/ReportLocalWaypoint.h"
#include "openjaus/mobility/Triggers/SetTravelSpeed.h"
#include "openjaus/mobility/Triggers/QueryTravelSpeed.h"
#include "openjaus/mobility/Triggers/ReportTravelSpeed.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace mobility
{

/// \class LocalWaypointDriver LocalWaypointDriver.h
/// \brief %LocalWaypointDriver Component implements the urn:jaus:jss:mobility:LocalWaypointDriver services.
/// The %LocalWaypointDriver component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%LocalWaypointDriver Service</dt>
/// <dd><p>
/// The function of the Local Waypoint Driver is to move the platform given a single target waypoint, desired travel
/// speed, current platform pose and current velocity state. A single waypoint is provided via the Set Local Waypoint
/// message. The waypoint remains unchanged until a new Set Local Waypoint message is received. A waypoint consists of
/// the desired position and orientation of the platform. The second input consists of the desired travel speed. The
/// desired travel speed remains unchanged unless a new Set Travel Speed Message is received. The travel speed may then
/// be changed at any time during waypoint navigation. The travel speed is reset to zero for all transitions from the
/// Ready State.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:mobility:LocalWaypointDriver<br/><br/>
/// <b>Version:</b> 1.0<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Management</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT LocalWaypointDriver : public core::Managed, public mobility::LocalWaypointDriverInterface
{

public:
	LocalWaypointDriver();
	virtual ~LocalWaypointDriver();

	virtual void resetLwdTravelSpeed();

	/// \brief SetLocalWaypoint action with input SetLocalWaypoint.
	/// SetLocalWaypoint action with input SetLocalWaypoint.
	/// \param[in]  setLocalWaypoint - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setLocalWaypoint(SetLocalWaypoint *setLocalWaypoint);

	/// \brief SetLwdTravelSpeed action with input SetTravelSpeed.
	/// SetLwdTravelSpeed action with input SetTravelSpeed.
	/// \param[in]  setTravelSpeed - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool setLwdTravelSpeed(SetTravelSpeed *setTravelSpeed);

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


	/// \brief isControllingLwdClient condition.
	/// isControllingLwdClient condition.
	/// \param[in]  setTravelSpeed - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLwdClient(SetTravelSpeed *setTravelSpeed);

	/// \brief isControllingLwdClient condition.
	/// isControllingLwdClient condition.
	/// \param[in]  setLocalWaypoint - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isControllingLwdClient(SetLocalWaypoint *setLocalWaypoint);


	/// \brief waypointExists condition.
	/// waypointExists condition.
	/// \param[in]  setTravelSpeed - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool waypointExists(SetTravelSpeed *setTravelSpeed);


	// Start of user code for additional methods:
	// End of user code

protected:
	LwdDefaultLoop lwdDefaultLoop;
	LwdReadyLoop lwdReadyLoop;
	LwdControlledLoop lwdControlledLoop;


	// Start of user code for additional members:
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // LOCALWAYPOINTDRIVER_H
