/**
\file TimeRecord.h

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
#include "openjaus/core/Triggers/Fields/TimeRecord.h"

namespace openjaus
{
namespace core
{

TimeRecord::TimeRecord():
	time(),
	date()
{
	this->presenceVector = 0;

	fields.push_back(&time);
	time.setName("Time");
	time.setOptional(true);
	// Nothing

	fields.push_back(&date);
	date.setName("Date");
	date.setOptional(true);
	// Nothing

}

TimeRecord::TimeRecord(const TimeRecord &source)
{
	this->copy(const_cast<TimeRecord&>(source));
}

TimeRecord::~TimeRecord()
{

}


TimeBitField& TimeRecord::getTime(void)
{
	return this->time;
}

DateBitField& TimeRecord::getDate(void)
{
	return this->date;
}

int TimeRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	if(this->isTimeEnabled())
	{
		byteSize += dst->pack(time);
	}
	if(this->isDateEnabled())
	{
		byteSize += dst->pack(date);
	}
	return byteSize;
}
int TimeRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	if(this->isTimeEnabled())
	{
		byteSize += src->unpack(time);
	}
	if(this->isDateEnabled())
	{
		byteSize += src->unpack(date);
	}
	return byteSize;
}

int TimeRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	if(this->isTimeEnabled())
	{
		length += time.length(); // time
	}
	if(this->isDateEnabled())
	{
		length += date.length(); // date
	}
	return length;
}

std::string TimeRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"TimeRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTimeEnabled value=\"" << std::boolalpha << this->isTimeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isDateEnabled value=\"" << std::boolalpha << this->isDateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isTimeEnabled())
	{
		oss << time.toXml(level+1); // time
	}
	if(this->isDateEnabled())
	{
		oss << date.toXml(level+1); // date
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void TimeRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t TimeRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool TimeRecord::isTimeEnabled(void) const
{
	return (this->presenceVector & (0x01 << TimeRecord::TIME));
}

void TimeRecord::enableTime(void)
{
	this->presenceVector |= 0x01 << TimeRecord::TIME;
}

void TimeRecord::disableTime(void)
{
	this->presenceVector &= ~(0x01 << TimeRecord::TIME);
}

bool TimeRecord::isDateEnabled(void) const
{
	return (this->presenceVector & (0x01 << TimeRecord::DATE));
}

void TimeRecord::enableDate(void)
{
	this->presenceVector |= 0x01 << TimeRecord::DATE;
}

void TimeRecord::disableDate(void)
{
	this->presenceVector &= ~(0x01 << TimeRecord::DATE);
}


void TimeRecord::copy(TimeRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->time.copy(source.getTime()); 
 
	this->date.copy(source.getDate()); 
 
}

} // namespace core
} // namespace openjaus

