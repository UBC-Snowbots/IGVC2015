/**
\file Events.h

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

#ifndef EVENTS_SERVICE_INTERFACE_H
#define EVENTS_SERVICE_INTERFACE_H

#include <openjaus.h>
#include "openjaus/core/TransportInterface.h"
#include "openjaus/core/Triggers/CreateEvent.h"
#include "openjaus/core/Triggers/UpdateEvent.h"
#include "openjaus/core/Triggers/CancelEvent.h"
#include "openjaus/core/Triggers/ConfirmEventRequest.h"
#include "openjaus/core/Triggers/RejectEventRequest.h"
#include "openjaus/core/Triggers/CreateCommandEvent.h"
#include "openjaus/core/Triggers/QueryEvents.h"
#include "openjaus/core/Triggers/QueryEventTimeout.h"
#include "openjaus/core/Triggers/ReportEvents.h"
#include "openjaus/core/Triggers/Event.h"
#include "openjaus/core/Triggers/ReportEventTimeout.h"
#include "openjaus/core/Triggers/CommandEvent.h"
namespace openjaus
{
namespace core
{

/// \class EventsInterface EventsInterface.h
/// \brief Provides an abstract interface for the %Events service. 
/// <p>
/// This service is used to set up event notifications. Since this service does not contain any messages and data on
/// which events can be setup, it is useful only when derived by other services that contain messages and data on which
/// events can be defined.
/// </p><br/><br/>
/// <b>URI:</b> %urn:jaus:jss:core:Events<br/><br/>
/// <b>Version:</b> 1.1<br/><br/>
/// <b>Inherits From:</b><ul>
/// <li>urn:jaus:jss:core:Transport</li>
/// </ul>
/// </dd>

class OPENJAUS_EXPORT EventsInterface
{
public:
	virtual ~EventsInterface(){};
	
	/// \brief CreateEvent action with input CreateEvent.
	/// CreateEvent action with input CreateEvent.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool createEvent(CreateEvent *createEvent) = 0;

	/// \brief ResetEventTimer action with input CreateEvent.
	/// ResetEventTimer action with input CreateEvent.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool resetEventTimer(CreateEvent *createEvent) = 0;

	/// \brief ResetEventTimer action with input UpdateEvent.
	/// ResetEventTimer action with input UpdateEvent.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool resetEventTimer(UpdateEvent *updateEvent) = 0;

	/// \brief UpdateEvent action with input CreateEvent.
	/// UpdateEvent action with input CreateEvent.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateEvent(CreateEvent *createEvent) = 0;

	/// \brief UpdateEvent action with input UpdateEvent.
	/// UpdateEvent action with input UpdateEvent.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool updateEvent(UpdateEvent *updateEvent) = 0;

	/// \brief CancelEvent action with input CancelEvent.
	/// CancelEvent action with input CancelEvent.
	/// \param[in]  cancelEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool cancelEvent(CancelEvent *cancelEvent) = 0;

	/// \brief StopEventTimer action with input CancelEvent.
	/// StopEventTimer action with input CancelEvent.
	/// \param[in]  cancelEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool stopEventTimer(CancelEvent *cancelEvent) = 0;

	/// \brief Send action for RejectEventRequest with input message CreateEvent.
	/// Send action for RejectEventRequest with input message CreateEvent.
	/// \param[in] createEvent - Input Trigger.
	/// \return RejectEventRequest Output Message.
	virtual RejectEventRequest getRejectEventRequest(CreateEvent *createEvent) = 0;

	/// \brief Send action for RejectEventRequest with input message UpdateEvent.
	/// Send action for RejectEventRequest with input message UpdateEvent.
	/// \param[in] updateEvent - Input Trigger.
	/// \return RejectEventRequest Output Message.
	virtual RejectEventRequest getRejectEventRequest(UpdateEvent *updateEvent) = 0;

	/// \brief Send action for RejectEventRequest with input message CancelEvent.
	/// Send action for RejectEventRequest with input message CancelEvent.
	/// \param[in] cancelEvent - Input Trigger.
	/// \return RejectEventRequest Output Message.
	virtual RejectEventRequest getRejectEventRequest(CancelEvent *cancelEvent) = 0;

	/// \brief Send action for ConfirmEventRequest with input message CreateEvent.
	/// Send action for ConfirmEventRequest with input message CreateEvent.
	/// \param[in] createEvent - Input Trigger.
	/// \return ConfirmEventRequest Output Message.
	virtual ConfirmEventRequest getConfirmEventRequest(CreateEvent *createEvent) = 0;

	/// \brief Send action for ConfirmEventRequest with input message UpdateEvent.
	/// Send action for ConfirmEventRequest with input message UpdateEvent.
	/// \param[in] updateEvent - Input Trigger.
	/// \return ConfirmEventRequest Output Message.
	virtual ConfirmEventRequest getConfirmEventRequest(UpdateEvent *updateEvent) = 0;

	/// \brief Send action for ConfirmEventRequest with input message CancelEvent.
	/// Send action for ConfirmEventRequest with input message CancelEvent.
	/// \param[in] cancelEvent - Input Trigger.
	/// \return ConfirmEventRequest Output Message.
	virtual ConfirmEventRequest getConfirmEventRequest(CancelEvent *cancelEvent) = 0;

	/// \brief Send action for ReportEvents with input message QueryEvents.
	/// Send action for ReportEvents with input message QueryEvents.
	/// \param[in] queryEvents - Input Trigger.
	/// \return ReportEvents Output Message.
	virtual ReportEvents getReportEvents(QueryEvents *queryEvents) = 0;

	/// \brief ConfirmEvent action with input ConfirmEventRequest.
	/// ConfirmEvent action with input ConfirmEventRequest.
	/// \param[in]  confirmEventRequest - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool confirmEvent(ConfirmEventRequest *confirmEventRequest) = 0;

	/// \brief HandleIncomingEvent action with input Event.
	/// HandleIncomingEvent action with input Event.
	/// \param[in]  event - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool handleIncomingEvent(Event *event) = 0;

	/// \brief HandleReportEvents action with input ReportEvents.
	/// HandleReportEvents action with input ReportEvents.
	/// \param[in]  reportEvents - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
	virtual bool handleReportEvents(ReportEvents *reportEvents) = 0;

	/// \brief isSupported condition.
	/// isSupported condition.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isSupported(CreateEvent *createEvent) = 0;

	/// \brief isSupported condition.
	/// isSupported condition.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isSupported(UpdateEvent *updateEvent) = 0;

	/// \brief eventExists condition.
	/// eventExists condition.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool eventExists(CreateEvent *createEvent) = 0;

	/// \brief eventExists condition.
	/// eventExists condition.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool eventExists(UpdateEvent *updateEvent) = 0;

	/// \brief eventExists condition.
	/// eventExists condition.
	/// \param[in]  cancelEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool eventExists(CancelEvent *cancelEvent) = 0;

};

} // namespace core
} // namespace openjaus

#endif // EVENTS_SERVICE_INTERFACE_H
