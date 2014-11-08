

/**
\file AnalogFormatEnumeration.h

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
#include "openjaus/environment/Triggers/Fields/AnalogFormatEnumeration.h"

namespace openjaus
{
namespace environment
{

AnalogFormatEnumeration::AnalogFormatEnumeration() :
	value(static_cast<AnalogFormatEnumeration::AnalogFormatEnum>(0))
{
}

AnalogFormatEnumeration::~AnalogFormatEnumeration()
{

}

AnalogFormatEnumeration::AnalogFormatEnum AnalogFormatEnumeration::getValue(void) const
{
	return this->value;
}

void AnalogFormatEnumeration::setValue(AnalogFormatEnumeration::AnalogFormatEnum value)
{
	this->value = value;
}

int AnalogFormatEnumeration::to(system::Buffer *dst)
{
	return dst->pack(static_cast<uint8_t>(this->getValue()));
}

int AnalogFormatEnumeration::from(system::Buffer *src)
{
	int sizeBytes = 0;
	uint8_t intValue;
	sizeBytes = src->unpack(intValue);
	this->setValue(static_cast<AnalogFormatEnumeration::AnalogFormatEnum>(intValue));
	return sizeBytes;
}

int AnalogFormatEnumeration::length(void)
{
	return sizeof(uint8_t);
}

std::string AnalogFormatEnumeration::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Enumeration name=\"AnalogFormat\" intValue=\"" << this->getValue() << "\" strValue=\"" << this->toString() << "\" />\n";
	return oss.str();
}

std::string AnalogFormatEnumeration::toString() const
{
	switch(this->value)
	{
		case NTSC_M:
			return "NTSC_M";
		case NTSC_J:
			return "NTSC_J";
		case PAL_N:
			return "PAL_N";
		case PAL_M:
			return "PAL_M";
		case SECAM_L:
			return "SECAM_L";
		case SECAM_BG:
			return "SECAM_BG";
		default:
			return "Unknown";
	}
}

} // namespace environment
} // namespace openjaus

