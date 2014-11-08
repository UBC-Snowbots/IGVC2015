
/**
\file CommandEvent.h

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
#include "openjaus/core/Triggers/CommandEvent.h"

namespace openjaus
{
namespace core
{

CommandEvent::CommandEvent() : 
	model::Message(),
	eventID(),
	commandResult()
{
	this->id = CommandEvent::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of the event");
	eventID.setValue(0);

	fields.push_back(&commandResult);
	commandResult.setName("CommandResult");
	commandResult.setOptional(false);
	// Nothing to init

}

CommandEvent::CommandEvent(model::Message *message) :
	model::Message(message),
	eventID(),
	commandResult()
{
	this->id = CommandEvent::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);


	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique identifier of the event");
	eventID.setValue(0);

	fields.push_back(&commandResult);
	commandResult.setName("CommandResult");
	commandResult.setOptional(false);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

CommandEvent::~CommandEvent()
{

}


uint8_t CommandEvent::getEventID(void)
{
	return this->eventID.getValue();
}

void CommandEvent::setEventID(uint8_t value)
{
	this->eventID.setValue(value);
}

CommandResultEnumeration::CommandResultEnum CommandEvent::getCommandResult(void)
{
	return this->commandResult.getValue();
}

void CommandEvent::setCommandResult(CommandResultEnumeration::CommandResultEnum value)
{
	this->commandResult.setValue(value);
}

int CommandEvent::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(eventID);
	byteSize += dst->pack(commandResult);
	return byteSize;
}

int CommandEvent::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(eventID);
	byteSize += src->unpack(commandResult);
	return byteSize;
}

int CommandEvent::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += eventID.length(); // eventID
	length += commandResult.length(); // commandResult
	return length;
}

std::string CommandEvent::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"CommandEvent\"";
	oss << " id=\"0x41F6\" >\n";
	oss << eventID.toXml(level+1); // eventID
	oss << commandResult.toXml(level+1); // commandResult
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus


