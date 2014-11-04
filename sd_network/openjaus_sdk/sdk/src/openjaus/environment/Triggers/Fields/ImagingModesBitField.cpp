

/**
\file ImagingModesBitField.h

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
#include "openjaus/environment/Triggers/Fields/ImagingModesBitField.h"

namespace openjaus
{
namespace environment
{

ImagingModesBitField::ImagingModesBitField() : 
	color(),
	greyscale(),
	infrared(),
	lowlight()
{

}

ImagingModesBitField::~ImagingModesBitField()
{

}

bool ImagingModesBitField::setColor(bool value)
{
	this->color = value;
	return true;
}

bool ImagingModesBitField::getColor(void) const
{
	return this->color;
}


bool ImagingModesBitField::setGreyscale(bool value)
{
	this->greyscale = value;
	return true;
}

bool ImagingModesBitField::getGreyscale(void) const
{
	return this->greyscale;
}


bool ImagingModesBitField::setInfrared(bool value)
{
	this->infrared = value;
	return true;
}

bool ImagingModesBitField::getInfrared(void) const
{
	return this->infrared;
}


bool ImagingModesBitField::setLowlight(bool value)
{
	this->lowlight = value;
	return true;
}

bool ImagingModesBitField::getLowlight(void) const
{
	return this->lowlight;
}



int ImagingModesBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->color & ImagingModesBitField::COLOR_BIT_MASK) << ImagingModesBitField::COLOR_START_BIT);
	intValue |= ((this->greyscale & ImagingModesBitField::GREYSCALE_BIT_MASK) << ImagingModesBitField::GREYSCALE_START_BIT);
	intValue |= ((this->infrared & ImagingModesBitField::INFRARED_BIT_MASK) << ImagingModesBitField::INFRARED_START_BIT);
	intValue |= ((this->lowlight & ImagingModesBitField::LOWLIGHT_BIT_MASK) << ImagingModesBitField::LOWLIGHT_START_BIT);
	return dst->pack(intValue);
}

int ImagingModesBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->color = (intValue >> (ImagingModesBitField::COLOR_START_BIT)) & ImagingModesBitField::COLOR_BIT_MASK;
	this->greyscale = (intValue >> (ImagingModesBitField::GREYSCALE_START_BIT)) & ImagingModesBitField::GREYSCALE_BIT_MASK;
	this->infrared = (intValue >> (ImagingModesBitField::INFRARED_START_BIT)) & ImagingModesBitField::INFRARED_BIT_MASK;
	this->lowlight = (intValue >> (ImagingModesBitField::LOWLIGHT_START_BIT)) & ImagingModesBitField::LOWLIGHT_BIT_MASK;

	return byteSize;
}

int ImagingModesBitField::length(void)
{
	return sizeof(uint8_t);
}

void ImagingModesBitField::copy(ImagingModesBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setColor(source.getColor());
	setGreyscale(source.getGreyscale());
	setInfrared(source.getInfrared());
	setLowlight(source.getLowlight());

}

std::string ImagingModesBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->color & ImagingModesBitField::COLOR_BIT_MASK) << ImagingModesBitField::COLOR_START_BIT);
	intValue |= ((this->greyscale & ImagingModesBitField::GREYSCALE_BIT_MASK) << ImagingModesBitField::GREYSCALE_START_BIT);
	intValue |= ((this->infrared & ImagingModesBitField::INFRARED_BIT_MASK) << ImagingModesBitField::INFRARED_START_BIT);
	intValue |= ((this->lowlight & ImagingModesBitField::LOWLIGHT_BIT_MASK) << ImagingModesBitField::LOWLIGHT_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Color\" value=\"" << getColor() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Greyscale\" value=\"" << getGreyscale() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Infrared\" value=\"" << getInfrared() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"Lowlight\" value=\"" << getLowlight() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

