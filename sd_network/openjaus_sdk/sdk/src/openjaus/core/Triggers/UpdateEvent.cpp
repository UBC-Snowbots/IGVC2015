
/**
\file UpdateEvent.h

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

#include <openjaus.h>
#include "openjaus/core/Triggers/UpdateEvent.h"

namespace openjaus
{
namespace core
{

UpdateEvent::UpdateEvent() : 
	model::Message(),
	requestID(),
	eventType(),
	requestedPeriodicRate_Hz(),
	eventID(),
	queryMessage()
{
	this->id = UpdateEvent::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setValue(0);

	fields.push_back(&eventType);
	eventType.setName("EventType");
	eventType.setOptional(false);
	eventType.setInterpretation("The kind of event to create");
	// Nothing to init

	fields.push_back(&requestedPeriodicRate_Hz);
	requestedPeriodicRate_Hz.setName("RequestedPeriodicRate");
	requestedPeriodicRate_Hz.setOptional(false);
	requestedPeriodicRate_Hz.setInterpretation("Must be specified for periodic event, and set to 0 for every change ");
	// Nothing to init

	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of existing event to update");
	eventID.setValue(0);

	fields.push_back(&queryMessage);
	queryMessage.setName("QueryMessage");
	queryMessage.setOptional(false);
	queryMessage.setInterpretation("The JAUS Query message  to be used by the receiving component to generate the report message(s)");
	// Nothing to Init

}

UpdateEvent::UpdateEvent(model::Message *message) :
	model::Message(message),
	requestID(),
	eventType(),
	requestedPeriodicRate_Hz(),
	eventID(),
	queryMessage()
{
	this->id = UpdateEvent::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setValue(0);

	fields.push_back(&eventType);
	eventType.setName("EventType");
	eventType.setOptional(false);
	eventType.setInterpretation("The kind of event to create");
	// Nothing to init

	fields.push_back(&requestedPeriodicRate_Hz);
	requestedPeriodicRate_Hz.setName("RequestedPeriodicRate");
	requestedPeriodicRate_Hz.setOptional(false);
	requestedPeriodicRate_Hz.setInterpretation("Must be specified for periodic event, and set to 0 for every change ");
	// Nothing to init

	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of existing event to update");
	eventID.setValue(0);

	fields.push_back(&queryMessage);
	queryMessage.setName("QueryMessage");
	queryMessage.setOptional(false);
	queryMessage.setInterpretation("The JAUS Query message  to be used by the receiving component to generate the report message(s)");
	// Nothing to Init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

UpdateEvent::~UpdateEvent()
{

}


uint8_t UpdateEvent::getRequestID(void)
{
	return this->requestID.getValue();
}

void UpdateEvent::setRequestID(uint8_t value)
{
	this->requestID.setValue(value);
}

EventTypeEnumeration::EventTypeEnum UpdateEvent::getEventType(void)
{
	return this->eventType.getValue();
}

void UpdateEvent::setEventType(EventTypeEnumeration::EventTypeEnum value)
{
	this->eventType.setValue(value);
}

double UpdateEvent::getRequestedPeriodicRate_Hz(void)
{
	return this->requestedPeriodicRate_Hz.getValue();
}

void UpdateEvent::setRequestedPeriodicRate_Hz(double value)
{
	this->requestedPeriodicRate_Hz.setValue(value);
}

uint8_t UpdateEvent::getEventID(void)
{
	return this->eventID.getValue();
}

void UpdateEvent::setEventID(uint8_t value)
{
	this->eventID.setValue(value);
}

model::Message *UpdateEvent::getQueryMessage(void)
{
	model::Message *msg = this->queryMessage.getMessage();
	msg->setAckNak(this->getAckNak());
	msg->setDestination(this->getDestination());
	msg->setSequenceNumber(this->getSequenceNumber());
	msg->setSource(this->getSource());
	msg->setTimeStamp(this->getTimeStamp());
	msg->setType(transport::JAUS_MESSAGE);

	return msg;
}

void UpdateEvent::setQueryMessage(model::Message* message)
{
	this->queryMessage.setMessage(message);
}

int UpdateEvent::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(requestID);
	byteSize += dst->pack(eventType);
	byteSize += dst->pack(requestedPeriodicRate_Hz);
	byteSize += dst->pack(eventID);
	byteSize += dst->pack(queryMessage);
	return byteSize;
}

int UpdateEvent::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(requestID);
	byteSize += src->unpack(eventType);
	byteSize += src->unpack(requestedPeriodicRate_Hz);
	byteSize += src->unpack(eventID);
	byteSize += src->unpack(queryMessage);
	return byteSize;
}

int UpdateEvent::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += requestID.length(); // requestID
	length += eventType.length(); // eventType
	length += requestedPeriodicRate_Hz.length(); // requestedPeriodicRate_Hz
	length += eventID.length(); // eventID
	length += queryMessage.length(); // queryMessage
	return length;
}

std::string UpdateEvent::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"UpdateEvent\"";
	oss << " id=\"0x01F1\" >\n";
	oss << requestID.toXml(level+1); // requestID
	oss << eventType.toXml(level+1); // eventType
	oss << requestedPeriodicRate_Hz.toXml(level+1); // requestedPeriodicRate_Hz
	oss << eventID.toXml(level+1); // eventID
	oss << queryMessage.toXml(level+1); // queryMessage
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


