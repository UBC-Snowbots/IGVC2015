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

#ifndef QUERYEVENTSVARIANT_H
#define QUERYEVENTSVARIANT_H

#include <openjaus.h>
#include "openjaus/core/Triggers/Fields/EventTypeEnumeration.h"

namespace openjaus
{
namespace core
{

class OPENJAUS_EXPORT QueryEventsVariant : public openjaus::model::fields::Variant
{
public:

	QueryEventsVariant();
	~QueryEventsVariant();

	enum TypeEnum {MESSAGEID = 0, EVENTTYPE = 1, EVENTID = 2, ALLEVENTS = 3};
	QueryEventsVariant::TypeEnum getType(void);
	void setType(QueryEventsVariant::TypeEnum type);

	void copy(QueryEventsVariant& source);
	virtual int to(system::Buffer *dst);
	virtual int from(system::Buffer *src);
	virtual int length(void);
	std::string toXml(unsigned char level=0) const;
		

	uint16_t getMessageID(void);
	void setMessageID(uint16_t value);

	EventTypeEnumeration::EventTypeEnum getEventType(void);
	void setEventType(EventTypeEnumeration::EventTypeEnum value);

	uint8_t getEventID(void);
	void setEventID(uint8_t value);

	uint8_t getAllEvents(void);
	void setAllEvents(uint8_t value);

	std::string typeToString(void);

private:
	TypeEnum type;
	
	model::fields::UnsignedShort messageID;
	EventTypeEnumeration eventType;
	model::fields::UnsignedByte eventID;
	model::fields::UnsignedByte allEvents;
};

} // namespace core
} // namespace openjaus

#endif // QUERYEVENTSVARIANT_H

