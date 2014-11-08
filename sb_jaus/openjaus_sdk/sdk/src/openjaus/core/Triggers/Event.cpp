
/**
\file Event.h

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
#include "openjaus/core/Triggers/Event.h"

namespace openjaus
{
namespace core
{

Event::Event() : 
	model::Message(),
	eventID(),
	sequenceNumber(),
	reportMessage()
{
	this->id = Event::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of the enclosed event");
	eventID.setValue(0);

	fields.push_back(&sequenceNumber);
	sequenceNumber.setName("SequenceNumber");
	sequenceNumber.setOptional(false);
	sequenceNumber.setInterpretation("Sequential count of the number of times this event has been issued");
	sequenceNumber.setValue(0);

	fields.push_back(&reportMessage);
	reportMessage.setName("ReportMessage");
	reportMessage.setOptional(false);
	// Nothing to Init

}

Event::Event(model::Message *message) :
	model::Message(message),
	eventID(),
	sequenceNumber(),
	reportMessage()
{
	this->id = Event::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of the enclosed event");
	eventID.setValue(0);

	fields.push_back(&sequenceNumber);
	sequenceNumber.setName("SequenceNumber");
	sequenceNumber.setOptional(false);
	sequenceNumber.setInterpretation("Sequential count of the number of times this event has been issued");
	sequenceNumber.setValue(0);

	fields.push_back(&reportMessage);
	reportMessage.setName("ReportMessage");
	reportMessage.setOptional(false);
	// Nothing to Init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

Event::~Event()
{

}


uint8_t Event::getEventID(void)
{
	return this->eventID.getValue();
}

void Event::setEventID(uint8_t value)
{
	this->eventID.setValue(value);
}

uint8_t Event::getSequenceNumber(void)
{
	return this->sequenceNumber.getValue();
}

void Event::setSequenceNumber(uint8_t value)
{
	this->sequenceNumber.setValue(value);
}

model::Message *Event::getReportMessage(void)
{
	model::Message *msg = this->reportMessage.getMessage();
	msg->setAckNak(this->getAckNak());
	msg->setDestination(this->getDestination());
	msg->setSequenceNumber(this->getSequenceNumber());
	msg->setSource(this->getSource());
	msg->setTimeStamp(this->getTimeStamp());
	msg->setType(transport::JAUS_MESSAGE);

	return msg;
}

void Event::setReportMessage(model::Message* message)
{
	this->reportMessage.setMessage(message);
}

int Event::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(eventID);
	byteSize += dst->pack(sequenceNumber);
	byteSize += dst->pack(reportMessage);
	return byteSize;
}

int Event::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(eventID);
	byteSize += src->unpack(sequenceNumber);
	byteSize += src->unpack(reportMessage);
	return byteSize;
}

int Event::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += eventID.length(); // eventID
	length += sequenceNumber.length(); // sequenceNumber
	length += reportMessage.length(); // reportMessage
	return length;
}

std::string Event::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"Event\"";
	oss << " id=\"0x41F1\" >\n";
	oss << eventID.toXml(level+1); // eventID
	oss << sequenceNumber.toXml(level+1); // sequenceNumber
	oss << reportMessage.toXml(level+1); // reportMessage
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


