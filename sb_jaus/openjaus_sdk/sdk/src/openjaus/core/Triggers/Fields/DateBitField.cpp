

/**
\file DateBitField.h

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
#include "openjaus/core/Triggers/Fields/DateBitField.h"

namespace openjaus
{
namespace core
{

DateBitField::DateBitField() : 
	day(),
	month(),
	year()
{

}

DateBitField::~DateBitField()
{

}

bool DateBitField::setDay(long value)
{
	if(value < DAY_MIN_VALUE || value > DAY_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->day = value;
	return true;
}
    
long DateBitField::getDay(void) const
{
	return 	this->day;
}

bool DateBitField::setMonth(long value)
{
	if(value < MONTH_MIN_VALUE || value > MONTH_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->month = value;
	return true;
}
    
long DateBitField::getMonth(void) const
{
	return 	this->month;
}

bool DateBitField::setYear(long value)
{
	if(value < YEAR_MIN_VALUE || value > YEAR_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->year = value;
	return true;
}
    
long DateBitField::getYear(void) const
{
	return 	this->year;
}


int DateBitField::to(system::Buffer *dst)
{
	uint16_t intValue = 0;

	intValue |= ((this->day & DateBitField::DAY_BIT_MASK) << DateBitField::DAY_START_BIT);
	intValue |= ((this->month & DateBitField::MONTH_BIT_MASK) << DateBitField::MONTH_START_BIT);
	intValue |= ((this->year & DateBitField::YEAR_BIT_MASK) << DateBitField::YEAR_START_BIT);
	return dst->pack(intValue);
}

int DateBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint16_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->day = (intValue >> (DateBitField::DAY_START_BIT)) & DateBitField::DAY_BIT_MASK;
	this->month = (intValue >> (DateBitField::MONTH_START_BIT)) & DateBitField::MONTH_BIT_MASK;
	this->year = (intValue >> (DateBitField::YEAR_START_BIT)) & DateBitField::YEAR_BIT_MASK;

	return byteSize;
}

int DateBitField::length(void)
{
	return sizeof(uint16_t);
}

void DateBitField::copy(DateBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setDay(source.getDay());
	setMonth(source.getMonth());
	setYear(source.getYear());

}

std::string DateBitField::toXml(unsigned char level) const
{
	uint16_t intValue = 0;

	intValue |= ((this->day & DateBitField::DAY_BIT_MASK) << DateBitField::DAY_START_BIT);
	intValue |= ((this->month & DateBitField::MONTH_BIT_MASK) << DateBitField::MONTH_START_BIT);
	intValue |= ((this->year & DateBitField::YEAR_BIT_MASK) << DateBitField::YEAR_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"Day\" value=\"" << getDay() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"Month\" value=\"" << getMonth() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"Year\" value=\"" << getYear() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus

