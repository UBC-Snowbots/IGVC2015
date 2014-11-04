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

#ifndef EVENTS_COMPONENT_H
#define EVENTS_COMPONENT_H

#include <openjaus.h>
#include "openjaus/core/Transport.h"
#include "openjaus/core/EventsInterface.h"
#include "openjaus/core/Transitions/EventsLoop.h"
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
// Start of user code for additional headers:
#include "openjaus/core/Triggers/Fields/EventTypeEnumeration.h"
#include <algorithm>
// End of user code

namespace openjaus
{
namespace core
{

/// \class Events Events.h
/// \brief %Events Component implements the urn:jaus:jss:core:Events services.
/// The %Events component provides an implementation of the following service(s). This component can 
/// be extended for customized functionality or used via instantiation.
/// <dl>
/// <dt>%Events Service</dt>
/// <dd><p>
/// This service is used to set up event notifications. Since this service does not contain any messages and data on
/// which events can be setup, it is useful only when derived by other services that contain messages and data on which
/// events can be defined.
/// </p><br/><br/>
/// <b>URI:</b> urn:jaus:jss:core:Events<br/><br/>
/// <b>Version:</b> 1.1<br/>
/// <dl><dt><b>Inherits From:</b></dt>
/// <dd>urn:jaus:jss:core:Transport</dd>
/// </dl></dd>
/// </dl>
class OPENJAUS_EXPORT Events : public core::Transport, public core::EventsInterface
{

public:
	Events();
	virtual ~Events();

	/// \brief CreateEvent action with input CreateEvent.
	/// CreateEvent action with input CreateEvent.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool createEvent(CreateEvent *createEvent);

	/// \brief ResetEventTimer action with input CreateEvent.
	/// ResetEventTimer action with input CreateEvent.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool resetEventTimer(CreateEvent *createEvent);
    
	/// \brief ResetEventTimer action with input UpdateEvent.
	/// ResetEventTimer action with input UpdateEvent.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool resetEventTimer(UpdateEvent *updateEvent);

	/// \brief UpdateEvent action with input CreateEvent.
	/// UpdateEvent action with input CreateEvent.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateEvent(CreateEvent *createEvent);
    
	/// \brief UpdateEvent action with input UpdateEvent.
	/// UpdateEvent action with input UpdateEvent.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool updateEvent(UpdateEvent *updateEvent);

	/// \brief CancelEvent action with input CancelEvent.
	/// CancelEvent action with input CancelEvent.
	/// \param[in]  cancelEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool cancelEvent(CancelEvent *cancelEvent);

	/// \brief StopEventTimer action with input CancelEvent.
	/// StopEventTimer action with input CancelEvent.
	/// \param[in]  cancelEvent - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool stopEventTimer(CancelEvent *cancelEvent);

	/// \brief Send action for RejectEventRequest with input message CreateEvent.
	/// Send action for RejectEventRequest with input message CreateEvent.
	/// \param[in] createEvent - Input Trigger.
	/// \return RejectEventRequest Output Message.
	virtual RejectEventRequest getRejectEventRequest(CreateEvent *createEvent);

    
	/// \brief Send action for RejectEventRequest with input message UpdateEvent.
	/// Send action for RejectEventRequest with input message UpdateEvent.
	/// \param[in] updateEvent - Input Trigger.
	/// \return RejectEventRequest Output Message.
	virtual RejectEventRequest getRejectEventRequest(UpdateEvent *updateEvent);

    
	/// \brief Send action for RejectEventRequest with input message CancelEvent.
	/// Send action for RejectEventRequest with input message CancelEvent.
	/// \param[in] cancelEvent - Input Trigger.
	/// \return RejectEventRequest Output Message.
	virtual RejectEventRequest getRejectEventRequest(CancelEvent *cancelEvent);

	/// \brief Send action for ConfirmEventRequest with input message CreateEvent.
	/// Send action for ConfirmEventRequest with input message CreateEvent.
	/// \param[in] createEvent - Input Trigger.
	/// \return ConfirmEventRequest Output Message.
	virtual ConfirmEventRequest getConfirmEventRequest(CreateEvent *createEvent);

    
	/// \brief Send action for ConfirmEventRequest with input message UpdateEvent.
	/// Send action for ConfirmEventRequest with input message UpdateEvent.
	/// \param[in] updateEvent - Input Trigger.
	/// \return ConfirmEventRequest Output Message.
	virtual ConfirmEventRequest getConfirmEventRequest(UpdateEvent *updateEvent);

    
	/// \brief Send action for ConfirmEventRequest with input message CancelEvent.
	/// Send action for ConfirmEventRequest with input message CancelEvent.
	/// \param[in] cancelEvent - Input Trigger.
	/// \return ConfirmEventRequest Output Message.
	virtual ConfirmEventRequest getConfirmEventRequest(CancelEvent *cancelEvent);

	/// \brief Send action for ReportEvents with input message QueryEvents.
	/// Send action for ReportEvents with input message QueryEvents.
	/// \param[in] queryEvents - Input Trigger.
	/// \return ReportEvents Output Message.
	virtual ReportEvents getReportEvents(QueryEvents *queryEvents);

	/// \brief ConfirmEvent action with input ConfirmEventRequest.
	/// ConfirmEvent action with input ConfirmEventRequest.
	/// \param[in]  confirmEventRequest - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool confirmEvent(ConfirmEventRequest *confirmEventRequest);

	/// \brief HandleIncomingEvent action with input Event.
	/// HandleIncomingEvent action with input Event.
	/// \param[in]  event - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool handleIncomingEvent(Event *event);

	/// \brief HandleReportEvents action with input ReportEvents.
	/// HandleReportEvents action with input ReportEvents.
	/// \param[in]  reportEvents - Input Trigger.
	/// \return Whether the message was properly processed by this action. 
    virtual bool handleReportEvents(ReportEvents *reportEvents);


	/// \brief isSupported condition.
	/// isSupported condition.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isSupported(CreateEvent *createEvent);

	/// \brief isSupported condition.
	/// isSupported condition.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool isSupported(UpdateEvent *updateEvent);


	/// \brief eventExists condition.
	/// eventExists condition.
	/// \param[in]  createEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool eventExists(CreateEvent *createEvent);

	/// \brief eventExists condition.
	/// eventExists condition.
	/// \param[in]  updateEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool eventExists(UpdateEvent *updateEvent);

	/// \brief eventExists condition.
	/// eventExists condition.
	/// \param[in]  cancelEvent - Input Trigger.
	/// \return Whether the condition is true. 
	virtual bool eventExists(CancelEvent *cancelEvent);


	// Start of user code for additional methods:
	bool createOutgoingPeriodicEvent(uint8_t eventId, double rateHz, model::Message *query);
	bool createOutgoingOnChangeEvent(uint8_t eventId, model::Message *query);

	bool publish(model::Message *message);
	bool unpublish(model::Message *message);

	uint32_t subscribePeriodic(transport::Address& address, model::Message *query, double rate_Hz);
	uint32_t subscribeOnChange(transport::Address& address, model::Message *query);
	bool unsubscribe(uint32_t subscriptionId);
	// End of user code

protected:
	EventsLoop eventsLoop;


	// Start of user code for additional members:
	model::TriggerQueue changedQueue;
	std::map<system::Timer *, model::Connection *> periodicQueries;
	std::map<uint16_t, std::vector<model::Connection *> > onChangeTriggers;

	std::vector<uint16_t> publishedIds;
	system::Thread changedThread;

	std::map<uint32_t, model::Connection *> inConnections;
	std::map<uint8_t, model::Connection *> outConnections;

	std::map<uint8_t, uint32_t> inRequestIdMap;
	std::map<uint8_t, std::map<int, uint8_t> > outRequestIdMap;

	uint32_t localIdCount;

	void timerEvent(system::Timer *timer);
	void *changedEventThread();

	uint32_t subscribeIncoming(model::ConnectionType type, double rate_Hz, transport::Address& address, model::Message *query);
	uint8_t getAvailableUniqueEventId();
	// End of user code
};

} // namespace component
} // namespace openjaus

#endif // EVENTS_H
