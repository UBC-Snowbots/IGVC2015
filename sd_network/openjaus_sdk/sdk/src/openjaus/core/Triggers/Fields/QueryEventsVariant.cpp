

/**
\file QueryEventsVariant.h

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
#include "openjaus/core/Triggers/Fields/QueryEventsVariant.h"

namespace openjaus
{
namespace core
{

QueryEventsVariant::QueryEventsVariant() :
	type(),
	messageID(),
	eventType(),
	eventID(),
	allEvents()
{
}

QueryEventsVariant::~QueryEventsVariant()
{

}

void QueryEventsVariant::setType(QueryEventsVariant::TypeEnum type)
{
	this->type = type;
}

QueryEventsVariant::TypeEnum QueryEventsVariant::getType(void)
{
	return this->type;
}

std::string QueryEventsVariant::typeToString(void)
{
	switch(this->type)
	{
		case MESSAGEID:
			return "MESSAGEID";
		case EVENTTYPE:
			return "EVENTTYPE";
		case EVENTID:
			return "EVENTID";
		case ALLEVENTS:
			return "ALLEVENTS";
		default:
			return "Unknown";
	}
}

void QueryEventsVariant::copy(QueryEventsVariant& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	this->type = source.getType();
	switch(this->type)
	{
		case MESSAGEID:
			this->messageID.setName("MessageID");
			this->messageID.setOptional(false);
			this->messageID.setInterpretation("Query Message ID of the Event message that the receiving component is inquiring about. ");
			this->messageID.setValue(source.getMessageID());
			break;

		case EVENTTYPE:
			this->eventType.setName("EventTypeEnum");
			this->eventType.setOptional(false);
			this->eventType.setInterpretation("The kind of event to create");
			this->eventType.setValue(source.getEventType());
			break;

		case EVENTID:
			this->eventID.setName("EventID");
			this->eventID.setOptional(false);
			this->eventID.setInterpretation("Event ID returned by Confirm Event for details on specific event.");
			this->eventID.setValue(source.getEventID());
			break;

		case ALLEVENTS:
			this->allEvents.setName("AllEvents");
			this->allEvents.setOptional(false);
			this->allEvents.setInterpretation("All events should be reported.");
			this->allEvents.setValue(source.getAllEvents());
			break;

		default:
			THROW_EXCEPTION("Unknown QueryEventsVariant Type: " << this->type);
	}
}

int QueryEventsVariant::to(system::Buffer *dst)
{
	int result = 0;
	result += dst->pack(static_cast<uint8_t>(this->type));
	switch(this->type)
	{
		case MESSAGEID:
			result += dst->pack(messageID);
			return result;

		case EVENTTYPE:
			result += dst->pack(eventType);
			return result;

		case EVENTID:
			result += dst->pack(eventID);
			return result;

		case ALLEVENTS:
			result += dst->pack(allEvents);
			return result;

		default:
			THROW_EXCEPTION("Unknown QueryEventsVariant Type: " << this->type);
	}
}

int QueryEventsVariant::from(system::Buffer *src)
{
	int result = 0;

	uint8_t intValue;
	result += src->unpack(intValue);
	this->type = static_cast<QueryEventsVariant::TypeEnum>(intValue);

	switch(this->type)
	{
		case MESSAGEID:
			result += src->unpack(messageID);
			return result;

		case EVENTTYPE:
			result += src->unpack(eventType);
			return result;

		case EVENTID:
			result += src->unpack(eventID);
			return result;

		case ALLEVENTS:
			result += src->unpack(allEvents);
			return result;

		default:
			THROW_EXCEPTION("Unknown QueryEventsVariant Type: " << this->type);
	}
}

int QueryEventsVariant::length(void)
{
	int result = 0;
	result += sizeof(uint8_t); // Type
	switch(this->type)
	{
		case MESSAGEID:
			result += messageID.length();
			return result;

		case EVENTTYPE:
			result += eventType.length();
			return result;

		case EVENTID:
			result += eventID.length();
			return result;

		case ALLEVENTS:
			result += allEvents.length();
			return result;

		default:
			THROW_EXCEPTION("Unknown QueryEventsVariant Type: " << this->type);
	}
}

std::string QueryEventsVariant::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Variant name=\"" << this->name << "\">\n";

	switch(this->type)
	{
		case MESSAGEID:
			oss << prefix.str() << "\t" << "<type>MESSAGEID</type>\n";
			oss << messageID.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case EVENTTYPE:
			oss << prefix.str() << "\t" << "<type>EVENTTYPE</type>\n";
			oss << eventType.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case EVENTID:
			oss << prefix.str() << "\t" << "<type>EVENTID</type>\n";
			oss << eventID.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		case ALLEVENTS:
			oss << prefix.str() << "\t" << "<type>ALLEVENTS</type>\n";
			oss << allEvents.toXml(level+1);
			oss << prefix.str() << "</Variant>\n";
			return oss.str();

		default:
			THROW_EXCEPTION("Unknown QueryEventsVariant Type: " << this->type);
	}
}




uint16_t QueryEventsVariant::getMessageID(void)
{
	if(this->type != MESSAGEID)
	{
		LOG("Misinterpretation of Variant (QueryEventsVariant). Requesting object of type \"MESSAGEID\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->messageID.getValue();
}

void QueryEventsVariant::setMessageID(uint16_t value)
{
	this->messageID.setValue(value);
}

EventTypeEnumeration::EventTypeEnum QueryEventsVariant::getEventType(void)
{
	if(this->type != EVENTTYPE)
	{
		LOG("Misinterpretation of Variant (QueryEventsVariant). Requesting object of type \"EVENTTYPE\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->eventType.getValue();
}

void QueryEventsVariant::setEventType(EventTypeEnumeration::EventTypeEnum value)
{
	this->eventType.setValue(value);
}

uint8_t QueryEventsVariant::getEventID(void)
{
	if(this->type != EVENTID)
	{
		LOG("Misinterpretation of Variant (QueryEventsVariant). Requesting object of type \"EVENTID\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->eventID.getValue();
}

void QueryEventsVariant::setEventID(uint8_t value)
{
	this->type = EVENTID;
	this->eventID.setValue(value);
}

uint8_t QueryEventsVariant::getAllEvents(void)
{
	if(this->type != ALLEVENTS)
	{
		LOG("Misinterpretation of Variant (QueryEventsVariant). Requesting object of type \"ALLEVENTS\" while variant is of type \"" << this->typeToString() << "\"" << std::endl);
	}
	return this->allEvents.getValue();
}

void QueryEventsVariant::setAllEvents(uint8_t value)
{
	this->type = ALLEVENTS;
	this->allEvents.setValue(value);
}

} // namespace core
} // namespace openjaus

