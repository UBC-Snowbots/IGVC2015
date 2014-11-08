

/**
\file LightSensitivityLevelsBitField.h

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
#include "openjaus/environment/Triggers/Fields/LightSensitivityLevelsBitField.h"

namespace openjaus
{
namespace environment
{

LightSensitivityLevelsBitField::LightSensitivityLevelsBitField() : 
	autoISO(),
	iSO100(),
	iSO200(),
	iSO400(),
	iSO800(),
	iSO1600(),
	iSO3200()
{

}

LightSensitivityLevelsBitField::~LightSensitivityLevelsBitField()
{

}

bool LightSensitivityLevelsBitField::setAutoISO(bool value)
{
	this->autoISO = value;
	return true;
}

bool LightSensitivityLevelsBitField::getAutoISO(void) const
{
	return this->autoISO;
}


bool LightSensitivityLevelsBitField::setISO100(bool value)
{
	this->iSO100 = value;
	return true;
}

bool LightSensitivityLevelsBitField::getISO100(void) const
{
	return this->iSO100;
}


bool LightSensitivityLevelsBitField::setISO200(bool value)
{
	this->iSO200 = value;
	return true;
}

bool LightSensitivityLevelsBitField::getISO200(void) const
{
	return this->iSO200;
}


bool LightSensitivityLevelsBitField::setISO400(bool value)
{
	this->iSO400 = value;
	return true;
}

bool LightSensitivityLevelsBitField::getISO400(void) const
{
	return this->iSO400;
}


bool LightSensitivityLevelsBitField::setISO800(bool value)
{
	this->iSO800 = value;
	return true;
}

bool LightSensitivityLevelsBitField::getISO800(void) const
{
	return this->iSO800;
}


bool LightSensitivityLevelsBitField::setISO1600(bool value)
{
	this->iSO1600 = value;
	return true;
}

bool LightSensitivityLevelsBitField::getISO1600(void) const
{
	return this->iSO1600;
}


bool LightSensitivityLevelsBitField::setISO3200(bool value)
{
	this->iSO3200 = value;
	return true;
}

bool LightSensitivityLevelsBitField::getISO3200(void) const
{
	return this->iSO3200;
}



int LightSensitivityLevelsBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->autoISO & LightSensitivityLevelsBitField::AUTOISO_BIT_MASK) << LightSensitivityLevelsBitField::AUTOISO_START_BIT);
	intValue |= ((this->iSO100 & LightSensitivityLevelsBitField::ISO100_BIT_MASK) << LightSensitivityLevelsBitField::ISO100_START_BIT);
	intValue |= ((this->iSO200 & LightSensitivityLevelsBitField::ISO200_BIT_MASK) << LightSensitivityLevelsBitField::ISO200_START_BIT);
	intValue |= ((this->iSO400 & LightSensitivityLevelsBitField::ISO400_BIT_MASK) << LightSensitivityLevelsBitField::ISO400_START_BIT);
	intValue |= ((this->iSO800 & LightSensitivityLevelsBitField::ISO800_BIT_MASK) << LightSensitivityLevelsBitField::ISO800_START_BIT);
	intValue |= ((this->iSO1600 & LightSensitivityLevelsBitField::ISO1600_BIT_MASK) << LightSensitivityLevelsBitField::ISO1600_START_BIT);
	intValue |= ((this->iSO3200 & LightSensitivityLevelsBitField::ISO3200_BIT_MASK) << LightSensitivityLevelsBitField::ISO3200_START_BIT);
	return dst->pack(intValue);
}

int LightSensitivityLevelsBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->autoISO = (intValue >> (LightSensitivityLevelsBitField::AUTOISO_START_BIT)) & LightSensitivityLevelsBitField::AUTOISO_BIT_MASK;
	this->iSO100 = (intValue >> (LightSensitivityLevelsBitField::ISO100_START_BIT)) & LightSensitivityLevelsBitField::ISO100_BIT_MASK;
	this->iSO200 = (intValue >> (LightSensitivityLevelsBitField::ISO200_START_BIT)) & LightSensitivityLevelsBitField::ISO200_BIT_MASK;
	this->iSO400 = (intValue >> (LightSensitivityLevelsBitField::ISO400_START_BIT)) & LightSensitivityLevelsBitField::ISO400_BIT_MASK;
	this->iSO800 = (intValue >> (LightSensitivityLevelsBitField::ISO800_START_BIT)) & LightSensitivityLevelsBitField::ISO800_BIT_MASK;
	this->iSO1600 = (intValue >> (LightSensitivityLevelsBitField::ISO1600_START_BIT)) & LightSensitivityLevelsBitField::ISO1600_BIT_MASK;
	this->iSO3200 = (intValue >> (LightSensitivityLevelsBitField::ISO3200_START_BIT)) & LightSensitivityLevelsBitField::ISO3200_BIT_MASK;

	return byteSize;
}

int LightSensitivityLevelsBitField::length(void)
{
	return sizeof(uint8_t);
}

void LightSensitivityLevelsBitField::copy(LightSensitivityLevelsBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setAutoISO(source.getAutoISO());
	setISO100(source.getISO100());
	setISO200(source.getISO200());
	setISO400(source.getISO400());
	setISO800(source.getISO800());
	setISO1600(source.getISO1600());
	setISO3200(source.getISO3200());

}

std::string LightSensitivityLevelsBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->autoISO & LightSensitivityLevelsBitField::AUTOISO_BIT_MASK) << LightSensitivityLevelsBitField::AUTOISO_START_BIT);
	intValue |= ((this->iSO100 & LightSensitivityLevelsBitField::ISO100_BIT_MASK) << LightSensitivityLevelsBitField::ISO100_START_BIT);
	intValue |= ((this->iSO200 & LightSensitivityLevelsBitField::ISO200_BIT_MASK) << LightSensitivityLevelsBitField::ISO200_START_BIT);
	intValue |= ((this->iSO400 & LightSensitivityLevelsBitField::ISO400_BIT_MASK) << LightSensitivityLevelsBitField::ISO400_START_BIT);
	intValue |= ((this->iSO800 & LightSensitivityLevelsBitField::ISO800_BIT_MASK) << LightSensitivityLevelsBitField::ISO800_START_BIT);
	intValue |= ((this->iSO1600 & LightSensitivityLevelsBitField::ISO1600_BIT_MASK) << LightSensitivityLevelsBitField::ISO1600_START_BIT);
	intValue |= ((this->iSO3200 & LightSensitivityLevelsBitField::ISO3200_BIT_MASK) << LightSensitivityLevelsBitField::ISO3200_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"AutoISO\" value=\"" << getAutoISO() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ISO100\" value=\"" << getISO100() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ISO200\" value=\"" << getISO200() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ISO400\" value=\"" << getISO400() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ISO800\" value=\"" << getISO800() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ISO1600\" value=\"" << getISO1600() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ISO3200\" value=\"" << getISO3200() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

