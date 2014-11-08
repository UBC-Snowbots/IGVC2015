

/**
\file SupportedStatesBitField.h

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
#include "openjaus/environment/Triggers/Fields/SupportedStatesBitField.h"

namespace openjaus
{
namespace environment
{

SupportedStatesBitField::SupportedStatesBitField() : 
	active(),
	standby(),
	off()
{

}

SupportedStatesBitField::~SupportedStatesBitField()
{

}

bool SupportedStatesBitField::setActive(bool value)
{
	this->active = value;
	return true;
}

bool SupportedStatesBitField::getActive(void) const
{
	return this->active;
}


bool SupportedStatesBitField::setStandby(bool value)
{
	this->standby = value;
	return true;
}

bool SupportedStatesBitField::getStandby(void) const
{
	return this->standby;
}


bool SupportedStatesBitField::setOff(bool value)
{
	this->off = value;
	return true;
}

bool SupportedStatesBitField::getOff(void) const
{
	return this->off;
}



int SupportedStatesBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->active & SupportedStatesBitField::ACTIVE_BIT_MASK) << SupportedStatesBitField::ACTIVE_START_BIT);
	intValue |= ((this->standby & SupportedStatesBitField::STANDBY_BIT_MASK) << SupportedStatesBitField::STANDBY_START_BIT);
	intValue |= ((this->off & SupportedStatesBitField::OFF_BIT_MASK) << SupportedStatesBitField::OFF_START_BIT);
	return dst->pack(intValue);
}

int SupportedStatesBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->active = (intValue >> (SupportedStatesBitField::ACTIVE_START_BIT)) & SupportedStatesBitField::ACTIVE_BIT_MASK;
	this->standby = (intValue >> (SupportedStatesBitField::STANDBY_START_BIT)) & SupportedStatesBitField::STANDBY_BIT_MASK;
	this->off = (intValue >> (SupportedStatesBitField::OFF_START_BIT)) & SupportedStatesBitField::OFF_BIT_MASK;

	return byteSize;
}

int SupportedStatesBitField::length(void)
{
	return sizeof(uint8_t);
}

void SupportedStatesBitField::copy(SupportedStatesBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setActive(source.getActive());
	setStandby(source.getStandby());
	setOff(source.getOff());

}

std::string SupportedStatesBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->active & SupportedStatesBitField::ACTIVE_BIT_MASK) << SupportedStatesBitField::ACTIVE_START_BIT);
	intValue |= ((this->standby & SupportedStatesBitField::STANDBY_BIT_MASK) << SupportedStatesBitField::STANDBY_START_BIT);
	intValue |= ((this->off & SupportedStatesBitField::OFF_BIT_MASK) << SupportedStatesBitField::OFF_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Active\" value=\"" << getActive() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Standby\" value=\"" << getStandby() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Off\" value=\"" << getOff() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

