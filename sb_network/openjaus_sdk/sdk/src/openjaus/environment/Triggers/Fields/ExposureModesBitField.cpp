

/**
\file ExposureModesBitField.h

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
#include "openjaus/environment/Triggers/Fields/ExposureModesBitField.h"

namespace openjaus
{
namespace environment
{

ExposureModesBitField::ExposureModesBitField() : 
	autoExposure(),
	manualExposure(),
	shutterPriority(),
	aperturePriority()
{

}

ExposureModesBitField::~ExposureModesBitField()
{

}

bool ExposureModesBitField::setAutoExposure(bool value)
{
	this->autoExposure = value;
	return true;
}

bool ExposureModesBitField::getAutoExposure(void) const
{
	return this->autoExposure;
}


bool ExposureModesBitField::setManualExposure(bool value)
{
	this->manualExposure = value;
	return true;
}

bool ExposureModesBitField::getManualExposure(void) const
{
	return this->manualExposure;
}


bool ExposureModesBitField::setShutterPriority(bool value)
{
	this->shutterPriority = value;
	return true;
}

bool ExposureModesBitField::getShutterPriority(void) const
{
	return this->shutterPriority;
}


bool ExposureModesBitField::setAperturePriority(bool value)
{
	this->aperturePriority = value;
	return true;
}

bool ExposureModesBitField::getAperturePriority(void) const
{
	return this->aperturePriority;
}



int ExposureModesBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->autoExposure & ExposureModesBitField::AUTOEXPOSURE_BIT_MASK) << ExposureModesBitField::AUTOEXPOSURE_START_BIT);
	intValue |= ((this->manualExposure & ExposureModesBitField::MANUALEXPOSURE_BIT_MASK) << ExposureModesBitField::MANUALEXPOSURE_START_BIT);
	intValue |= ((this->shutterPriority & ExposureModesBitField::SHUTTERPRIORITY_BIT_MASK) << ExposureModesBitField::SHUTTERPRIORITY_START_BIT);
	intValue |= ((this->aperturePriority & ExposureModesBitField::APERTUREPRIORITY_BIT_MASK) << ExposureModesBitField::APERTUREPRIORITY_START_BIT);
	return dst->pack(intValue);
}

int ExposureModesBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->autoExposure = (intValue >> (ExposureModesBitField::AUTOEXPOSURE_START_BIT)) & ExposureModesBitField::AUTOEXPOSURE_BIT_MASK;
	this->manualExposure = (intValue >> (ExposureModesBitField::MANUALEXPOSURE_START_BIT)) & ExposureModesBitField::MANUALEXPOSURE_BIT_MASK;
	this->shutterPriority = (intValue >> (ExposureModesBitField::SHUTTERPRIORITY_START_BIT)) & ExposureModesBitField::SHUTTERPRIORITY_BIT_MASK;
	this->aperturePriority = (intValue >> (ExposureModesBitField::APERTUREPRIORITY_START_BIT)) & ExposureModesBitField::APERTUREPRIORITY_BIT_MASK;

	return byteSize;
}

int ExposureModesBitField::length(void)
{
	return sizeof(uint8_t);
}

void ExposureModesBitField::copy(ExposureModesBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setAutoExposure(source.getAutoExposure());
	setManualExposure(source.getManualExposure());
	setShutterPriority(source.getShutterPriority());
	setAperturePriority(source.getAperturePriority());

}

std::string ExposureModesBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->autoExposure & ExposureModesBitField::AUTOEXPOSURE_BIT_MASK) << ExposureModesBitField::AUTOEXPOSURE_START_BIT);
	intValue |= ((this->manualExposure & ExposureModesBitField::MANUALEXPOSURE_BIT_MASK) << ExposureModesBitField::MANUALEXPOSURE_START_BIT);
	intValue |= ((this->shutterPriority & ExposureModesBitField::SHUTTERPRIORITY_BIT_MASK) << ExposureModesBitField::SHUTTERPRIORITY_START_BIT);
	intValue |= ((this->aperturePriority & ExposureModesBitField::APERTUREPRIORITY_BIT_MASK) << ExposureModesBitField::APERTUREPRIORITY_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"AutoExposure\" value=\"" << getAutoExposure() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ManualExposure\" value=\"" << getManualExposure() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"ShutterPriority\" value=\"" << getShutterPriority() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"AperturePriority\" value=\"" << getAperturePriority() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

