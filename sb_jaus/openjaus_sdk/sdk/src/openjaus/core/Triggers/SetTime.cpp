
/**
\file SetTime.h

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
#include "openjaus/core/Triggers/SetTime.h"

namespace openjaus
{
namespace core
{

SetTime::SetTime() : 
	model::Message(),
	time(),
	date()
{
	this->id = SetTime::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	fields.push_back(&time);
	time.setName("Time");
	time.setOptional(true);
	// Nothing

	fields.push_back(&date);
	date.setName("Date");
	date.setOptional(true);
	// Nothing

}

SetTime::SetTime(model::Message *message) :
	model::Message(message),
	time(),
	date()
{
	this->id = SetTime::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&time);
	time.setName("Time");
	time.setOptional(true);
	// Nothing

	fields.push_back(&date);
	date.setName("Date");
	date.setOptional(true);
	// Nothing


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

SetTime::~SetTime()
{

}


TimeBitField& SetTime::getTime(void)
{
	return this->time;
}

DateBitField& SetTime::getDate(void)
{
	return this->date;
}

int SetTime::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
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

int SetTime::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
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

int SetTime::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
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

std::string SetTime::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetTime\"";
	oss << " id=\"0x0011\" >\n";
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
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void SetTime::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t SetTime::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool SetTime::isTimeEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetTime::TIME));
}

void SetTime::enableTime(void)
{
	this->presenceVector |= 0x01 << SetTime::TIME;
}

void SetTime::disableTime(void)
{
	this->presenceVector &= ~(0x01 << SetTime::TIME);
}

bool SetTime::isDateEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetTime::DATE));
}

void SetTime::enableDate(void)
{
	this->presenceVector |= 0x01 << SetTime::DATE;
}

void SetTime::disableDate(void)
{
	this->presenceVector &= ~(0x01 << SetTime::DATE);
}

} // namespace core
} // namespace openjaus


