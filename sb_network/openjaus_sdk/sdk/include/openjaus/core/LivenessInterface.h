/**
\file Liveness.h

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

#ifndef LIVENESS_SERVICE_INTERFACE_H
#define LIVENESS_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/EventsInterface.h"
#include "openjaus/core/Triggers/QueryHeartbeatPulse.h"
#include "openjaus/core/Triggers/ReportHeartbeatPulse.h"
namespace openjaus
{
namespace core
{

/// \class LivenessInterface LivenessInterface.h
/// \brief Provides an abstract interface for the %Liveness service. 
/// <p>
/// This service provides a means to maintain connection liveness between communicating components.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:core:Liveness<br/><br/>
/// <b>Version:</b> 1.0<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Events</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT LivenessInterface
{
public:
	virtual ~LivenessInterface(){};
	
	/// \brief Send a Report Heartbeat Pulse message to the component that sent the query
	/// Send a Report Heartbeat Pulse message to the component that sent the query
	/// \param[in] queryHeartbeatPulse - Input Trigger.
	/// \return ReportHeartbeatPulse Output Message.
	virtual ReportHeartbeatPulse getReportHeartbeatPulse(QueryHeartbeatPulse *queryHeartbeatPulse) = 0;

	/// \brief Processes ReportHeatbeat Messages to see if a new JAUS entity exists
	/// Processes ReportHeatbeat Messages to see if a new JAUS entity exists
	/// \param[in]  reportHeartbeatPulse - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool processHeartbeat(ReportHeartbeatPulse *reportHeartbeatPulse) = 0;

};

} // namespace core
} // namespace openjaus

#endif // LIVENESS_SERVICE_INTERFACE_H
