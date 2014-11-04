

/**
\file WhiteBalanceBitField.h

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
#include "openjaus/environment/Triggers/Fields/WhiteBalanceBitField.h"

namespace openjaus
{
namespace environment
{

WhiteBalanceBitField::WhiteBalanceBitField() : 
	autoWhiteBalance(),
	daylight(),
	cloudy(),
	shade(),
	tungsten(),
	flurescent(),
	flash()
{

}

WhiteBalanceBitField::~WhiteBalanceBitField()
{

}

bool WhiteBalanceBitField::setAutoWhiteBalance(bool value)
{
	this->autoWhiteBalance = value;
	return true;
}

bool WhiteBalanceBitField::getAutoWhiteBalance(void) const
{
	return this->autoWhiteBalance;
}


bool WhiteBalanceBitField::setDaylight(bool value)
{
	this->daylight = value;
	return true;
}

bool WhiteBalanceBitField::getDaylight(void) const
{
	return this->daylight;
}


bool WhiteBalanceBitField::setCloudy(bool value)
{
	this->cloudy = value;
	return true;
}

bool WhiteBalanceBitField::getCloudy(void) const
{
	return this->cloudy;
}


bool WhiteBalanceBitField::setShade(bool value)
{
	this->shade = value;
	return true;
}

bool WhiteBalanceBitField::getShade(void) const
{
	return this->shade;
}


bool WhiteBalanceBitField::setTungsten(bool value)
{
	this->tungsten = value;
	return true;
}

bool WhiteBalanceBitField::getTungsten(void) const
{
	return this->tungsten;
}


bool WhiteBalanceBitField::setFlurescent(bool value)
{
	this->flurescent = value;
	return true;
}

bool WhiteBalanceBitField::getFlurescent(void) const
{
	return this->flurescent;
}


bool WhiteBalanceBitField::setFlash(bool value)
{
	this->flash = value;
	return true;
}

bool WhiteBalanceBitField::getFlash(void) const
{
	return this->flash;
}



int WhiteBalanceBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->autoWhiteBalance & WhiteBalanceBitField::AUTOWHITEBALANCE_BIT_MASK) << WhiteBalanceBitField::AUTOWHITEBALANCE_START_BIT);
	intValue |= ((this->daylight & WhiteBalanceBitField::DAYLIGHT_BIT_MASK) << WhiteBalanceBitField::DAYLIGHT_START_BIT);
	intValue |= ((this->cloudy & WhiteBalanceBitField::CLOUDY_BIT_MASK) << WhiteBalanceBitField::CLOUDY_START_BIT);
	intValue |= ((this->shade & WhiteBalanceBitField::SHADE_BIT_MASK) << WhiteBalanceBitField::SHADE_START_BIT);
	intValue |= ((this->tungsten & WhiteBalanceBitField::TUNGSTEN_BIT_MASK) << WhiteBalanceBitField::TUNGSTEN_START_BIT);
	intValue |= ((this->flurescent & WhiteBalanceBitField::FLURESCENT_BIT_MASK) << WhiteBalanceBitField::FLURESCENT_START_BIT);
	intValue |= ((this->flash & WhiteBalanceBitField::FLASH_BIT_MASK) << WhiteBalanceBitField::FLASH_START_BIT);
	return dst->pack(intValue);
}

int WhiteBalanceBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->autoWhiteBalance = (intValue >> (WhiteBalanceBitField::AUTOWHITEBALANCE_START_BIT)) & WhiteBalanceBitField::AUTOWHITEBALANCE_BIT_MASK;
	this->daylight = (intValue >> (WhiteBalanceBitField::DAYLIGHT_START_BIT)) & WhiteBalanceBitField::DAYLIGHT_BIT_MASK;
	this->cloudy = (intValue >> (WhiteBalanceBitField::CLOUDY_START_BIT)) & WhiteBalanceBitField::CLOUDY_BIT_MASK;
	this->shade = (intValue >> (WhiteBalanceBitField::SHADE_START_BIT)) & WhiteBalanceBitField::SHADE_BIT_MASK;
	this->tungsten = (intValue >> (WhiteBalanceBitField::TUNGSTEN_START_BIT)) & WhiteBalanceBitField::TUNGSTEN_BIT_MASK;
	this->flurescent = (intValue >> (WhiteBalanceBitField::FLURESCENT_START_BIT)) & WhiteBalanceBitField::FLURESCENT_BIT_MASK;
	this->flash = (intValue >> (WhiteBalanceBitField::FLASH_START_BIT)) & WhiteBalanceBitField::FLASH_BIT_MASK;

	return byteSize;
}

int WhiteBalanceBitField::length(void)
{
	return sizeof(uint8_t);
}

void WhiteBalanceBitField::copy(WhiteBalanceBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setAutoWhiteBalance(source.getAutoWhiteBalance());
	setDaylight(source.getDaylight());
	setCloudy(source.getCloudy());
	setShade(source.getShade());
	setTungsten(source.getTungsten());
	setFlurescent(source.getFlurescent());
	setFlash(source.getFlash());

}

std::string WhiteBalanceBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->autoWhiteBalance & WhiteBalanceBitField::AUTOWHITEBALANCE_BIT_MASK) << WhiteBalanceBitField::AUTOWHITEBALANCE_START_BIT);
	intValue |= ((this->daylight & WhiteBalanceBitField::DAYLIGHT_BIT_MASK) << WhiteBalanceBitField::DAYLIGHT_START_BIT);
	intValue |= ((this->cloudy & WhiteBalanceBitField::CLOUDY_BIT_MASK) << WhiteBalanceBitField::CLOUDY_START_BIT);
	intValue |= ((this->shade & WhiteBalanceBitField::SHADE_BIT_MASK) << WhiteBalanceBitField::SHADE_START_BIT);
	intValue |= ((this->tungsten & WhiteBalanceBitField::TUNGSTEN_BIT_MASK) << WhiteBalanceBitField::TUNGSTEN_START_BIT);
	intValue |= ((this->flurescent & WhiteBalanceBitField::FLURESCENT_BIT_MASK) << WhiteBalanceBitField::FLURESCENT_START_BIT);
	intValue |= ((this->flash & WhiteBalanceBitField::FLASH_BIT_MASK) << WhiteBalanceBitField::FLASH_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"AutoWhiteBalance\" value=\"" << getAutoWhiteBalance() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Daylight\" value=\"" << getDaylight() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Cloudy\" value=\"" << getCloudy() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Shade\" value=\"" << getShade() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Tungsten\" value=\"" << getTungsten() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Flurescent\" value=\"" << getFlurescent() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Flash\" value=\"" << getFlash() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

