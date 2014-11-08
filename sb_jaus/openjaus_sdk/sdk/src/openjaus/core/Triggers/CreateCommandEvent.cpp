
/**
\file CreateCommandEvent.h

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
#include "openjaus/core/Triggers/CreateCommandEvent.h"

namespace openjaus
{
namespace core
{

CreateCommandEvent::CreateCommandEvent() : 
	model::Message(),
	requestID(),
	maximumAllowedDuration_sec(),
	commandMessage()
{
	this->id = CreateCommandEvent::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setInterpretation("ID of the event maintenance request (Create, Update, or Cancel)");
	requestID.setValue(0);

	fields.push_back(&maximumAllowedDuration_sec);
	maximumAllowedDuration_sec.setName("MaximumAllowedDuration");
	maximumAllowedDuration_sec.setOptional(false);
	maximumAllowedDuration_sec.setInterpretation("Any commands not executed within the maximum allowed time are considered a failure.");
	maximumAllowedDuration_sec.setValue(0);

	fields.push_back(&commandMessage);
	commandMessage.setName("CommandMessage");
	commandMessage.setOptional(false);
	// Nothing to Init

}

CreateCommandEvent::CreateCommandEvent(model::Message *message) :
	model::Message(message),
	requestID(),
	maximumAllowedDuration_sec(),
	commandMessage()
{
	this->id = CreateCommandEvent::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&requestID);
	requestID.setName("RequestID");
	requestID.setOptional(false);
	requestID.setInterpretation("ID of the event maintenance request (Create, Update, or Cancel)");
	requestID.setValue(0);

	fields.push_back(&maximumAllowedDuration_sec);
	maximumAllowedDuration_sec.setName("MaximumAllowedDuration");
	maximumAllowedDuration_sec.setOptional(false);
	maximumAllowedDuration_sec.setInterpretation("Any commands not executed within the maximum allowed time are considered a failure.");
	maximumAllowedDuration_sec.setValue(0);

	fields.push_back(&commandMessage);
	commandMessage.setName("CommandMessage");
	commandMessage.setOptional(false);
	// Nothing to Init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

CreateCommandEvent::~CreateCommandEvent()
{

}


int8_t CreateCommandEvent::getRequestID(void)
{
	return this->requestID.getValue();
}

void CreateCommandEvent::setRequestID(int8_t value)
{
	this->requestID.setValue(value);
}

uint32_t CreateCommandEvent::getMaximumAllowedDuration_sec(void)
{
	return this->maximumAllowedDuration_sec.getValue();
}

void CreateCommandEvent::setMaximumAllowedDuration_sec(uint32_t value)
{
	this->maximumAllowedDuration_sec.setValue(value);
}

model::Message *CreateCommandEvent::getCommandMessage(void)
{
	model::Message *msg = this->commandMessage.getMessage();
	msg->setAckNak(this->getAckNak());
	msg->setDestination(this->getDestination());
	msg->setSequenceNumber(this->getSequenceNumber());
	msg->setSource(this->getSource());
	msg->setTimeStamp(this->getTimeStamp());
	msg->setType(transport::JAUS_MESSAGE);

	return msg;
}

void CreateCommandEvent::setCommandMessage(model::Message* message)
{
	this->commandMessage.setMessage(message);
}

int CreateCommandEvent::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(requestID);
	byteSize += dst->pack(maximumAllowedDuration_sec);
	byteSize += dst->pack(commandMessage);
	return byteSize;
}

int CreateCommandEvent::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(requestID);
	byteSize += src->unpack(maximumAllowedDuration_sec);
	byteSize += src->unpack(commandMessage);
	return byteSize;
}

int CreateCommandEvent::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += requestID.length(); // requestID
	length += maximumAllowedDuration_sec.length(); // maximumAllowedDuration_sec
	length += commandMessage.length(); // commandMessage
	return length;
}

std::string CreateCommandEvent::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"CreateCommandEvent\"";
	oss << " id=\"0x01F6\" >\n";
	oss << requestID.toXml(level+1); // requestID
	oss << maximumAllowedDuration_sec.toXml(level+1); // maximumAllowedDuration_sec
	oss << commandMessage.toXml(level+1); // commandMessage
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


