

/**
\file ZoomModesBitField.h

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
#include "openjaus/environment/Triggers/Fields/ZoomModesBitField.h"

namespace openjaus
{
namespace environment
{

ZoomModesBitField::ZoomModesBitField() : 
	mixedZoom(),
	analogZoomOnly(),
	digitalZoomOnly(),
	noZoom()
{

}

ZoomModesBitField::~ZoomModesBitField()
{

}

bool ZoomModesBitField::setMixedZoom(bool value)
{
	this->mixedZoom = value;
	return true;
}

bool ZoomModesBitField::getMixedZoom(void) const
{
	return this->mixedZoom;
}


bool ZoomModesBitField::setAnalogZoomOnly(bool value)
{
	this->analogZoomOnly = value;
	return true;
}

bool ZoomModesBitField::getAnalogZoomOnly(void) const
{
	return this->analogZoomOnly;
}


bool ZoomModesBitField::setDigitalZoomOnly(bool value)
{
	this->digitalZoomOnly = value;
	return true;
}

bool ZoomModesBitField::getDigitalZoomOnly(void) const
{
	return this->digitalZoomOnly;
}


bool ZoomModesBitField::setNoZoom(bool value)
{
	this->noZoom = value;
	return true;
}

bool ZoomModesBitField::getNoZoom(void) const
{
	return this->noZoom;
}



int ZoomModesBitField::to(system::Buffer *dst)
{
	uint8_t intValue = 0;

	intValue |= ((this->mixedZoom & ZoomModesBitField::MIXEDZOOM_BIT_MASK) << ZoomModesBitField::MIXEDZOOM_START_BIT);
	intValue |= ((this->analogZoomOnly & ZoomModesBitField::ANALOGZOOMONLY_BIT_MASK) << ZoomModesBitField::ANALOGZOOMONLY_START_BIT);
	intValue |= ((this->digitalZoomOnly & ZoomModesBitField::DIGITALZOOMONLY_BIT_MASK) << ZoomModesBitField::DIGITALZOOMONLY_START_BIT);
	intValue |= ((this->noZoom & ZoomModesBitField::NOZOOM_BIT_MASK) << ZoomModesBitField::NOZOOM_START_BIT);
	return dst->pack(intValue);
}

int ZoomModesBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint8_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->mixedZoom = (intValue >> (ZoomModesBitField::MIXEDZOOM_START_BIT)) & ZoomModesBitField::MIXEDZOOM_BIT_MASK;
	this->analogZoomOnly = (intValue >> (ZoomModesBitField::ANALOGZOOMONLY_START_BIT)) & ZoomModesBitField::ANALOGZOOMONLY_BIT_MASK;
	this->digitalZoomOnly = (intValue >> (ZoomModesBitField::DIGITALZOOMONLY_START_BIT)) & ZoomModesBitField::DIGITALZOOMONLY_BIT_MASK;
	this->noZoom = (intValue >> (ZoomModesBitField::NOZOOM_START_BIT)) & ZoomModesBitField::NOZOOM_BIT_MASK;

	return byteSize;
}

int ZoomModesBitField::length(void)
{
	return sizeof(uint8_t);
}

void ZoomModesBitField::copy(ZoomModesBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setMixedZoom(source.getMixedZoom());
	setAnalogZoomOnly(source.getAnalogZoomOnly());
	setDigitalZoomOnly(source.getDigitalZoomOnly());
	setNoZoom(source.getNoZoom());

}

std::string ZoomModesBitField::toXml(unsigned char level) const
{
	uint8_t intValue = 0;

	intValue |= ((this->mixedZoom & ZoomModesBitField::MIXEDZOOM_BIT_MASK) << ZoomModesBitField::MIXEDZOOM_START_BIT);
	intValue |= ((this->analogZoomOnly & ZoomModesBitField::ANALOGZOOMONLY_BIT_MASK) << ZoomModesBitField::ANALOGZOOMONLY_START_BIT);
	intValue |= ((this->digitalZoomOnly & ZoomModesBitField::DIGITALZOOMONLY_BIT_MASK) << ZoomModesBitField::DIGITALZOOMONLY_START_BIT);
	intValue |= ((this->noZoom & ZoomModesBitField::NOZOOM_BIT_MASK) << ZoomModesBitField::NOZOOM_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"MixedZoom\" value=\"" << getMixedZoom() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"AnalogZoomOnly\" value=\"" << getAnalogZoomOnly() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"DigitalZoomOnly\" value=\"" << getDigitalZoomOnly() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldEnumeration name=\"NoZoom\" value=\"" << getNoZoom() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace environment
} // namespace openjaus

