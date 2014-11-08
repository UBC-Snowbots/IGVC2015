/**
\file ReportEventRecord.h

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
#include "openjaus/core/Triggers/Fields/ReportEventRecord.h"

namespace openjaus
{
namespace core
{

ReportEventRecord::ReportEventRecord():
	eventType(),
	eventID(),
	queryMessage()
{

	fields.push_back(&eventType);
	eventType.setName("EventType");
	eventType.setOptional(false);
	eventType.setInterpretation("The kind of event to create");
	// Nothing to init

	fields.push_back(&eventID);
	eventID.setName("EventID");
	eventID.setOptional(false);
	eventID.setInterpretation("Unique Identifier of existing event");
	eventID.setValue(0);

	fields.push_back(&queryMessage);
	queryMessage.setName("QueryMessage");
	queryMessage.setOptional(false);
	queryMessage.setInterpretation("The JAUS Query message to be used by the receiving component to generate the report message(s)");
	// Nothing to Init

}

ReportEventRecord::ReportEventRecord(const ReportEventRecord &source)
{
	this->copy(const_cast<ReportEventRecord&>(source));
}

ReportEventRecord::~ReportEventRecord()
{

}


EventTypeEnumeration::EventTypeEnum ReportEventRecord::getEventType(void)
{
	return this->eventType.getValue();
}

void ReportEventRecord::setEventType(EventTypeEnumeration::EventTypeEnum value)
{
	this->eventType.setValue(value);
}

uint8_t ReportEventRecord::getEventID(void)
{
	return this->eventID.getValue();
}

void ReportEventRecord::setEventID(uint8_t value)
{
	this->eventID.setValue(value);
}

model::Message *ReportEventRecord::getQueryMessage(void)
{
	model::Message *msg = this->queryMessage.getMessage();
	msg->setType(transport::JAUS_MESSAGE);

	return msg;
}

void ReportEventRecord::setQueryMessage(model::Message* message)
{
	this->queryMessage.setMessage(message);
}

int ReportEventRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(eventType);
	byteSize += dst->pack(eventID);
	byteSize += dst->pack(queryMessage);
	return byteSize;
}
int ReportEventRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(eventType);
	byteSize += src->unpack(eventID);
	byteSize += src->unpack(queryMessage);
	return byteSize;
}

int ReportEventRecord::length(void)
{
	int length = 0;
	length += eventType.length(); // eventType
	length += eventID.length(); // eventID
	length += queryMessage.length(); // queryMessage
	return length;
}

std::string ReportEventRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"ReportEventRecord\">\n";
	oss << eventType.toXml(level+1); // eventType
	oss << eventID.toXml(level+1); // eventID
	oss << queryMessage.toXml(level+1); // queryMessage
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}



void ReportEventRecord::copy(ReportEventRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->eventType.setName("EventTypeEnum");
	this->eventType.setOptional(false);
	this->eventType.setInterpretation("The kind of event to create");
	this->eventType.setValue(source.getEventType()); 
 
	this->eventID.setName("EventID");
	this->eventID.setOptional(false);
	this->eventID.setInterpretation("Unique Identifier of existing event");
	this->eventID.setValue(source.getEventID()); 
 
	this->queryMessage.copy(source.getQueryMessage()); 
 
}

} // namespace core
} // namespace openjaus

