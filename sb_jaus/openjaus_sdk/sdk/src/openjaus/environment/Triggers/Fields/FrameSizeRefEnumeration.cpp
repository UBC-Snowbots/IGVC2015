

/**
\file FrameSizeRefEnumeration.h

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
#include "openjaus/environment/Triggers/Fields/FrameSizeRefEnumeration.h"

namespace openjaus
{
namespace environment
{

FrameSizeRefEnumeration::FrameSizeRefEnumeration() :
	value(static_cast<FrameSizeRefEnumeration::FrameSizeRefEnum>(0))
{
}

FrameSizeRefEnumeration::~FrameSizeRefEnumeration()
{

}

FrameSizeRefEnumeration::FrameSizeRefEnum FrameSizeRefEnumeration::getValue(void) const
{
	return this->value;
}

void FrameSizeRefEnumeration::setValue(FrameSizeRefEnumeration::FrameSizeRefEnum value)
{
	this->value = value;
}

int FrameSizeRefEnumeration::to(system::Buffer *dst)
{
	return dst->pack(static_cast<uint8_t>(this->getValue()));
}

int FrameSizeRefEnumeration::from(system::Buffer *src)
{
	int sizeBytes = 0;
	uint8_t intValue;
	sizeBytes = src->unpack(intValue);
	this->setValue(static_cast<FrameSizeRefEnumeration::FrameSizeRefEnum>(intValue));
	return sizeBytes;
}

int FrameSizeRefEnumeration::length(void)
{
	return sizeof(uint8_t);
}

std::string FrameSizeRefEnumeration::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Enumeration name=\"FrameSizeRef\" intValue=\"" << this->getValue() << "\" strValue=\"" << this->toString() << "\" />\n";
	return oss.str();
}

std::string FrameSizeRefEnumeration::toString() const
{
	switch(this->value)
	{
		case SQCIF_128X96:
			return "sqcif_128x96";
		case QCIF_176X144:
			return "qcif_176x144";
		case CIF_352X288:
			return "cif_352x288";
		case CIF4_704X576:
			return "cif4_704x576";
		case CIF16_1408X1152:
			return "cif16_1408x1152";
		case QQVGA_160X120:
			return "qqvga_160x120";
		case QVGA_320X240:
			return "qvga_320x240";
		case VGA_640X480:
			return "vga_640x480";
		case SVGA_800X600:
			return "svga_800x600";
		case XGA_1024X768:
			return "xga_1024x768";
		case UXGA_1600X1200:
			return "uxga_1600x1200";
		case QXGA_2048X1536:
			return "qxga_2048x1536";
		case SXGA_1280X1024:
			return "sxga_1280x1024";
		case QSXGA_2560X2048:
			return "qsxga_2560x2048";
		case HSXGA_5120X4096:
			return "hsxga_5120x4096";
		case WVGA_852X480:
			return "wvga_852x480";
		case WXGA_1366X768:
			return "wxga_1366x768";
		case WSXGA_1600X1024:
			return "wsxga_1600x1024";
		case WUXGA_1920X1200:
			return "wuxga_1920x1200";
		case WOXGA_2560X1600:
			return "woxga_2560x1600";
		case WQSXGA_3200X2048:
			return "wqsxga_3200x2048";
		case WQUXGA_3840X2400:
			return "wquxga_3840x2400";
		case WHSXGA_6400X4096:
			return "whsxga_6400x4096";
		case WHUXGA_7680X4800:
			return "whuxga_7680x4800";
		case CGA_320X200:
			return "cga_320x200";
		case EGA_640X350:
			return "ega_640x350";
		case HD480_852X480:
			return "hd480_852x480";
		case HD720_1280X720:
			return "hd720_1280x720";
		case HD1080_1920X1080:
			return "hd1080_1920x1080";
		default:
			return "Unknown";
	}
}

} // namespace environment
} // namespace openjaus

