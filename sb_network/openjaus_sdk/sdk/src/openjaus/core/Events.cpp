/**
\file Events.cpp

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


#include "openjaus/core/Events.h"
// Start of user code for additional headers:
// End of user code

namespace openjaus
{
namespace core
{

Events::Events() : Transport(),
	eventsLoop()
{
	// Add Service Identification Data to implements list
	name = "Events";
	
	model::Service *eventsService = new model::Service();
	eventsService->setName("Events");
	eventsService->setUri("urn:jaus:jss:core:Events");
	eventsService->setVersionMajor(1);
	eventsService->setVersionMinor(1);
	this->implements->push_back(eventsService);
	
	

	eventsLoop.setInterface(this);
	eventsLoop.setTransportInterface(this);
	receive.addDefaultStateTransition(eventsLoop);
	
    
    
	// Start of user code for Constructor:
	localIdCount = 0;
	// End of user code
}

Events::~Events()
{
	// Start of user code for Destructor:
	// End of user code
}

bool Events::createEvent(CreateEvent *createEvent)
{
	// Start of user code for action createEvent(CreateEvent *createEvent):
	LOG_DEBUG("Got create event message");

	model::Message *query = createEvent->getQueryMessage();

	uint8_t uniqueEventId = getAvailableUniqueEventId();
	if(uniqueEventId == 255)
	{
		THROW_EXCEPTION("Cannot create outgoing connection. Maximum number of connections reached.");
	}

	outRequestIdMap[createEvent->getRequestID()][createEvent->getSource().getHash()] = uniqueEventId;

	outConnections[uniqueEventId] = new model::Connection();
	outConnections[uniqueEventId]->setLocalId(localIdCount);
	outConnections[uniqueEventId]->setEventId(uniqueEventId);
	outConnections[uniqueEventId]->setRequestId(createEvent->getRequestID());
	outConnections[uniqueEventId]->setAddress(createEvent->getSource());
	outConnections[uniqueEventId]->setQuery(query);

	if(createEvent->getEventType() == core::EventTypeEnumeration::PERIODIC)
	{
		outConnections[uniqueEventId]->setType(model::PERIODIC);
		outConnections[uniqueEventId]->setRate_Hz(createEvent->getRequestedPeriodicRate_Hz());

		this->createOutgoingPeriodicEvent(uniqueEventId, createEvent->getRequestedPeriodicRate_Hz(), query);
	}

	if(createEvent->getEventType() == core::EventTypeEnumeration::EVERY_CHANGE)
	{
		outConnections[uniqueEventId]->setType(model::ON_CHANGE);
		outConnections[uniqueEventId]->setRate_Hz(0);

		this->createOutgoingOnChangeEvent(uniqueEventId, query);
	}

	return true;
	// End of user code
}

bool Events::resetEventTimer(CreateEvent *createEvent)
{
	// Start of user code for action resetEventTimer(CreateEvent *createEvent):
	return false;
	// End of user code
}

bool Events::resetEventTimer(UpdateEvent *updateEvent)
{
	// Start of user code for action resetEventTimer(UpdateEvent *updateEvent):
	return false;
	// End of user code
}

bool Events::updateEvent(CreateEvent *createEvent)
{
	// Start of user code for action updateEvent(CreateEvent *createEvent):
	return false;
	// End of user code
}

bool Events::updateEvent(UpdateEvent *updateEvent)
{
	// Start of user code for action updateEvent(UpdateEvent *updateEvent):
	return false;
	// End of user code
}

bool Events::cancelEvent(CancelEvent *cancelEvent)
{
	// Start of user code for action cancelEvent(CancelEvent *cancelEvent):
	model::Connection *connection = outConnections[cancelEvent->getEventID()];

	if(connection->getType() == model::PERIODIC)
	{
		system::Timer *timer = connection->getTimer();
		timer->stop();
		periodicQueries.erase(timer);
		delete timer;
	}
	else
	{
		uint16_t responseId = outConnections[cancelEvent->getEventID()]->getResponseId();
		std::vector<model::Connection *> connections = onChangeTriggers[responseId];

		std::vector<model::Connection *>::iterator i = std::find(connections.begin(), connections.end(), connection);
		connections.erase(i);
	}

	outConnections.erase(cancelEvent->getEventID());
	delete connection->getQuery();
	delete connection;

	return false;
	// End of user code
}

bool Events::stopEventTimer(CancelEvent *cancelEvent)
{
	// Start of user code for action stopEventTimer(CancelEvent *cancelEvent):
	return false;
	// End of user code
}

core::RejectEventRequest Events::getRejectEventRequest(CreateEvent *createEvent)
{
	// Start of user code for action getRejectEventRequest(CreateEvent *createEvent):
	core::RejectEventRequest message;
	return message;
	// End of user code
}

core::RejectEventRequest Events::getRejectEventRequest(UpdateEvent *updateEvent)
{
	// Start of user code for action getRejectEventRequest(UpdateEvent *updateEvent):
	core::RejectEventRequest message;
	return message;
	// End of user code
}

core::RejectEventRequest Events::getRejectEventRequest(CancelEvent *cancelEvent)
{
	// Start of user code for action getRejectEventRequest(CancelEvent *cancelEvent):
	core::RejectEventRequest message;
	return message;
	// End of user code
}

core::ConfirmEventRequest Events::getConfirmEventRequest(CreateEvent *createEvent)
{
	// Start of user code for action getConfirmEventRequest(CreateEvent *createEvent):
	uint8_t uniqueEventId = outRequestIdMap[createEvent->getRequestID()][createEvent->getSource().getHash()];
	core::ConfirmEventRequest message;
	message.setRequestID(createEvent->getRequestID());
	message.setEventID(uniqueEventId);
	message.setConfirmedPeriodicRate_Hz(createEvent->getRequestedPeriodicRate_Hz());
	return message;
	// End of user code
}

core::ConfirmEventRequest Events::getConfirmEventRequest(UpdateEvent *updateEvent)
{
	// Start of user code for action getConfirmEventRequest(UpdateEvent *updateEvent):
	uint8_t uniqueEventId = outRequestIdMap[updateEvent->getRequestID()][updateEvent->getSource().getHash()];
	core::ConfirmEventRequest message;
	message.setRequestID(updateEvent->getRequestID());
	message.setEventID(uniqueEventId);
	message.setConfirmedPeriodicRate_Hz(updateEvent->getRequestedPeriodicRate_Hz());
	return message;
	// End of user code
}

core::ConfirmEventRequest Events::getConfirmEventRequest(CancelEvent *cancelEvent)
{
	// Start of user code for action getConfirmEventRequest(CancelEvent *cancelEvent):
	uint8_t uniqueEventId = outRequestIdMap[cancelEvent->getRequestID()][cancelEvent->getSource().getHash()];
	core::ConfirmEventRequest message;
	message.setRequestID(cancelEvent->getRequestID());
	message.setEventID(uniqueEventId);
	message.setConfirmedPeriodicRate_Hz(0);
	return message;
	// End of user code
}

core::ReportEvents Events::getReportEvents(QueryEvents *queryEvents)
{
	// Start of user code for action getReportEvents(QueryEvents *queryEvents):
	core::ReportEvents message;
	return message;
	// End of user code
}

bool Events::confirmEvent(ConfirmEventRequest *confirmEventRequest)
{
	// Start of user code for action confirmEvent(ConfirmEventRequest *confirmEventRequest):
	uint8_t requestId = confirmEventRequest->getRequestID();
	std::map<uint8_t, uint32_t>::iterator iRequest = inRequestIdMap.find(requestId);
	if(iRequest == inRequestIdMap.end())
	{
		LOG("Events: Received confirm event request for unrequested id: " << (int)requestId);
		return false;
	}

	uint32_t localId = iRequest->second;

	std::map<uint32_t, model::Connection *>::iterator iConnection = inConnections.find(localId);
	if(iConnection == inConnections.end())
	{
		LOG("Events: No connection found for local id: " << localId);
		return false;
	}

	// TODO: Because confirmEvent is received for both create and cancel requests we must keep track of the last request type
	// If it was cancel then erase the request, if it was create then do not erase the request and store info
	inConnections[localId]->setEventId(confirmEventRequest->getEventID());
	inConnections[localId]->setRate_Hz(confirmEventRequest->getConfirmedPeriodicRate_Hz());
	inConnections[localId]->setActive(true);
	inRequestIdMap.erase(requestId);
	return true;
	// End of user code
}

bool Events::handleIncomingEvent(Event *event)
{
	// Start of user code for action handleIncomingEvent(Event *event):
	LOG_DEBUG("Events: Received Incoming Event");

	model::Message *response = event->getReportMessage();
	receive.processTrigger(response);
	delete response;

	return true;
	// End of user code
}

bool Events::handleReportEvents(ReportEvents *reportEvents)
{
	// Start of user code for action handleReportEvents(ReportEvents *reportEvents):
	return false;
	// End of user code
}


bool Events::isSupported(CreateEvent *createEvent)
{
	// Start of user code for action isSupported(CreateEvent *createEvent):
	LOG_DEBUG("Events: Checking if create event is supported");

	if(	createEvent->getEventType() != core::EventTypeEnumeration::EVERY_CHANGE &&
		createEvent->getEventType() != core::EventTypeEnumeration::PERIODIC )
	{
		// Unsupported type
		LOG_DEBUG("Events: Event not supported, unknown event");
		return false;
	}

	// Check is state machine is responding to this query
	model::Message* query = createEvent->getQueryMessage();
	model::Message *response = receive.getResponse(query);
	delete query;
	if(!response)
	{
		LOG_DEBUG("Events: Event not supported, this component does not respond to request");
		return false;
	}

	// Check if published
	std::vector<uint16_t>::iterator i = std::find(publishedIds.begin(), publishedIds.end(), response->getId());
	if(i == publishedIds.end())
	{
		LOG_DEBUG("Events: Event not supported, requested ID is not published");
		return false;
	}

	delete response;

	return true;


	// End of user code
}

bool Events::isSupported(UpdateEvent *updateEvent)
{
	// Start of user code for action isSupported(UpdateEvent *updateEvent):
	return false;
	// End of user code
}


bool Events::eventExists(CreateEvent *createEvent)
{
	// Start of user code for action eventExists(CreateEvent *createEvent):
	if(outRequestIdMap.count(createEvent->getRequestID()) == 0)
	{
		return false;
	}

	if(outRequestIdMap[createEvent->getRequestID()].count(createEvent->getSource().getHash()) == 0)
	{
		return false;
	}

	uint8_t eventId = outRequestIdMap[createEvent->getRequestID()][createEvent->getSource().getHash()];

	if(outConnections.count(eventId) == 0)
	{
		return false;
	}

	return true;

	// End of user code
}

bool Events::eventExists(UpdateEvent *updateEvent)
{
	// Start of user code for action eventExists(UpdateEvent *updateEvent):
	if(outConnections.count(updateEvent->getEventID()) == 0)
	{
		return false;
	}

	return true;
	// End of user code
}

bool Events::eventExists(CancelEvent *cancelEvent)
{
	// Start of user code for action eventExists(CancelEvent *cancelEvent):
	if(outConnections.count(cancelEvent->getEventID()) == 0)
	{
		return false;
	}

	return true;
	// End of user code
}


// Start of user code for additional methods
void *Events::changedEventThread()
{
	while(changedThread.isRunning())
	{
		model::Trigger *response = changedQueue.pop();
		if(!response)
		{
			changedQueue.timedWait(500);
			continue;
		}

		std::vector<model::Connection *>::iterator i = onChangeTriggers[response->getId()].begin();
		while(i != onChangeTriggers[response->getId()].end())
		{
			model::Connection *connection = *i;

			model::Message* reportMessage = receive.getResponse(connection->getQuery());
			uint8_t sequenceNumber = connection->getSequenceNumber() + 1;
			connection->setSequenceNumber(sequenceNumber);

			core::Event *eventMsg = new core::Event();
			eventMsg->setDestination(connection->getAddress());
			eventMsg->setEventID(connection->getEventId());
			eventMsg->setSequenceNumber(sequenceNumber);
			eventMsg->setPayload(reportMessage);

			sendMessage(eventMsg);

			++i;
		}

	}

	return NULL;
}

bool Events::createOutgoingPeriodicEvent(uint8_t eventId, double rateHz, model::Message *query)
{
	LOG_DEBUG("Creating Outgoing Periodic Event");

	system::Timer *timer = new system::Timer(TIMER_METHOD(Events, timerEvent), this);
	timer->setInterval(static_cast<int>(1000.0 / rateHz));

	outConnections[eventId]->setTimer(timer);
	periodicQueries[timer] = outConnections[eventId];

	return true;
}

bool Events::createOutgoingOnChangeEvent(uint8_t eventId, model::Message *query)
{
	model::Message *response = receive.getResponse(query);
	if(response) // TODO: Throw error if no response
	{
		onChangeTriggers[response->getId()].push_back(outConnections[eventId]);

		outConnections[eventId]->setResponseId(response->getId());
	}

	return true;
}

void Events::timerEvent(system::Timer *timer)
{
	LOG_DEBUG("Periodic Event Timer Went Off");

	model::Connection *connection = periodicQueries[timer];
	model::Message* reportMessage = receive.getResponse(connection->getQuery());
	if(!reportMessage)
	{
		// TODO: Handle no response case (terminate connection?)
		LOG_DEBUG("No Response for Outgoing Periodic Event");
		return;
	}

	uint8_t sequenceNumber = connection->getSequenceNumber();
	sequenceNumber++;
	connection->setSequenceNumber(sequenceNumber);

	core::Event *eventMsg = new core::Event();
	LOG_DEBUG("Sending Event to: " << connection->getAddress());
	eventMsg->setDestination(connection->getAddress());
	eventMsg->setEventID(connection->getEventId());
	eventMsg->setReportMessage(reportMessage);
	eventMsg->setSequenceNumber(sequenceNumber);

	delete reportMessage;

	sendMessage(eventMsg);
}

// TODO: Change name so the message type is not ambiguous (i.e report, query, command)
bool Events::publish(model::Message *message)
{
	std::vector<uint16_t>::iterator i = std::find(publishedIds.begin(), publishedIds.end(), message->getId());
	if(i != publishedIds.end())
	{
		return true;
	}

	publishedIds.push_back(message->getId());
	message->setChangedQueue(&changedQueue);

	return true;
}

bool Events::unpublish(model::Message *message)
{
	std::vector<uint16_t>::iterator i = std::find(publishedIds.begin(), publishedIds.end(), message->getId());
	if(i != publishedIds.end())
	{
		message->setChangedQueue(NULL);
		publishedIds.erase(i);
		return false;
	}

	return false;
}

uint32_t Events::subscribePeriodic(transport::Address& address, model::Message *query, double rate_Hz)
{
	return subscribeIncoming(model::PERIODIC, rate_Hz, address, query);
}

uint32_t Events::subscribeOnChange(transport::Address& address, model::Message *query)
{
	return subscribeIncoming(model::ON_CHANGE, 0, address, query);
}

uint32_t Events::subscribeIncoming(model::ConnectionType type, double rate_Hz, transport::Address& address, model::Message *query)
{
	for(uint8_t requestId = 0; requestId < 255; ++requestId)
	{
		if(inRequestIdMap.count(requestId))
		{
			continue;
		}

		inRequestIdMap[requestId] = ++localIdCount;

		inConnections[localIdCount] = new model::Connection();
		inConnections[localIdCount]->setActive(false);
		inConnections[localIdCount]->setLocalId(localIdCount);
		inConnections[localIdCount]->setRequestId(requestId);
		inConnections[localIdCount]->setAddress(address);
		inConnections[localIdCount]->setType(type);
		inConnections[localIdCount]->setRate_Hz(rate_Hz);
		inConnections[localIdCount]->setQuery(query); // TODO: Make copy of query

		core::CreateEvent *createEvent = new core::CreateEvent();
		createEvent->setDestination(address);
		createEvent->setEventType(type == model::PERIODIC? core::EventTypeEnumeration::PERIODIC : core::EventTypeEnumeration::EVERY_CHANGE);
		createEvent->setRequestedPeriodicRate_Hz(rate_Hz); // TODO: Throw exception on bounds check
		createEvent->setRequestID(requestId);
		createEvent->setQueryMessage(query);

		sendMessage(createEvent);

		return localIdCount;
	}

	THROW_EXCEPTION("Cannot create subscription. Maximum number of sent requests has been reached.")
}

bool Events::unsubscribe(uint32_t subscriptionId)
{
	// If this connection has not been created, do not proceed
	if(!inConnections.count(subscriptionId))
	{
		LOG_DEBUG("Events: No connection exists for ID: " << subscriptionId);
		return false;
	}


	// Retrieve the connection from our map
	model::Connection *connection = inConnections[subscriptionId];

	// If this connection has been confirmed
	if(connection->isActive())
	{	// Send Cancel Event
		core::CancelEvent *cancelEvent = new core::CancelEvent();
		cancelEvent->setEventID(connection->getEventId());
		cancelEvent->setDestination(connection->getAddress());
		cancelEvent->setRequestID(connection->getRequestId());

		sendMessage(cancelEvent);
	}
	else
	{
		LOG_DEBUG("Events: Connection ID: " << subscriptionId << ", is not currently active");
		if(inRequestIdMap.count(connection->getRequestId()))
		{
			LOG_DEBUG("Events: Erasing connection request for connection: " << (int)connection->getRequestId());
			inRequestIdMap.erase(connection->getRequestId());
		}
	}

	// Destroy inConnection
	delete connection->getQuery();
	delete connection;
	inConnections[subscriptionId] = NULL;

	return true;
}

uint8_t Events::getAvailableUniqueEventId()
{
	uint8_t uniqueEventId = 0;
	for(uniqueEventId = 0; uniqueEventId < 255; ++uniqueEventId)
	{
		if(outConnections.count(uniqueEventId))
		{
			continue;
		}
		break;
	}
	return uniqueEventId;
}
// End of user code

} // namespace component
} // namespace openjaus

