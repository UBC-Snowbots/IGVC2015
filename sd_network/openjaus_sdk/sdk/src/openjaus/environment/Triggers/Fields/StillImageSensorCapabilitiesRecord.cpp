/**
\file StillImageSensorCapabilitiesRecord.h

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
#include "openjaus/environment/Triggers/Fields/StillImageSensorCapabilitiesRecord.h"

namespace openjaus
{
namespace environment
{

StillImageSensorCapabilitiesRecord::StillImageSensorCapabilitiesRecord():
	sensorID(),
	supportedFrameSizes(),
	supportedImageFormats()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&supportedFrameSizes);
	supportedFrameSizes.setName("SupportedFrameSizes");
	supportedFrameSizes.setOptional(true);
	// Nothing

	fields.push_back(&supportedImageFormats);
	supportedImageFormats.setName("SupportedImageFormats");
	supportedImageFormats.setOptional(true);
	// Nothing

}

StillImageSensorCapabilitiesRecord::StillImageSensorCapabilitiesRecord(const StillImageSensorCapabilitiesRecord &source)
{
	this->copy(const_cast<StillImageSensorCapabilitiesRecord&>(source));
}

StillImageSensorCapabilitiesRecord::~StillImageSensorCapabilitiesRecord()
{

}


uint16_t StillImageSensorCapabilitiesRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void StillImageSensorCapabilitiesRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

SupportedFrameSizesBitField& StillImageSensorCapabilitiesRecord::getSupportedFrameSizes(void)
{
	return this->supportedFrameSizes;
}

SupportedImageFormatsBitField& StillImageSensorCapabilitiesRecord::getSupportedImageFormats(void)
{
	return this->supportedImageFormats;
}

int StillImageSensorCapabilitiesRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	if(this->isSupportedFrameSizesEnabled())
	{
		byteSize += dst->pack(supportedFrameSizes);
	}
	if(this->isSupportedImageFormatsEnabled())
	{
		byteSize += dst->pack(supportedImageFormats);
	}
	return byteSize;
}
int StillImageSensorCapabilitiesRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	if(this->isSupportedFrameSizesEnabled())
	{
		byteSize += src->unpack(supportedFrameSizes);
	}
	if(this->isSupportedImageFormatsEnabled())
	{
		byteSize += src->unpack(supportedImageFormats);
	}
	return byteSize;
}

int StillImageSensorCapabilitiesRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += sensorID.length(); // sensorID
	if(this->isSupportedFrameSizesEnabled())
	{
		length += supportedFrameSizes.length(); // supportedFrameSizes
	}
	if(this->isSupportedImageFormatsEnabled())
	{
		length += supportedImageFormats.length(); // supportedImageFormats
	}
	return length;
}

std::string StillImageSensorCapabilitiesRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"StillImageSensorCapabilitiesRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedFrameSizesEnabled value=\"" << std::boolalpha << this->isSupportedFrameSizesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedImageFormatsEnabled value=\"" << std::boolalpha << this->isSupportedImageFormatsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	if(this->isSupportedFrameSizesEnabled())
	{
		oss << supportedFrameSizes.toXml(level+1); // supportedFrameSizes
	}
	if(this->isSupportedImageFormatsEnabled())
	{
		oss << supportedImageFormats.toXml(level+1); // supportedImageFormats
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void StillImageSensorCapabilitiesRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t StillImageSensorCapabilitiesRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool StillImageSensorCapabilitiesRecord::isSupportedFrameSizesEnabled(void) const
{
	return (this->presenceVector & (0x01 << StillImageSensorCapabilitiesRecord::SUPPORTEDFRAMESIZES));
}

void StillImageSensorCapabilitiesRecord::enableSupportedFrameSizes(void)
{
	this->presenceVector |= 0x01 << StillImageSensorCapabilitiesRecord::SUPPORTEDFRAMESIZES;
}

void StillImageSensorCapabilitiesRecord::disableSupportedFrameSizes(void)
{
	this->presenceVector &= ~(0x01 << StillImageSensorCapabilitiesRecord::SUPPORTEDFRAMESIZES);
}

bool StillImageSensorCapabilitiesRecord::isSupportedImageFormatsEnabled(void) const
{
	return (this->presenceVector & (0x01 << StillImageSensorCapabilitiesRecord::SUPPORTEDIMAGEFORMATS));
}

void StillImageSensorCapabilitiesRecord::enableSupportedImageFormats(void)
{
	this->presenceVector |= 0x01 << StillImageSensorCapabilitiesRecord::SUPPORTEDIMAGEFORMATS;
}

void StillImageSensorCapabilitiesRecord::disableSupportedImageFormats(void)
{
	this->presenceVector &= ~(0x01 << StillImageSensorCapabilitiesRecord::SUPPORTEDIMAGEFORMATS);
}


void StillImageSensorCapabilitiesRecord::copy(StillImageSensorCapabilitiesRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->supportedFrameSizes.copy(source.getSupportedFrameSizes()); 
 
	this->supportedImageFormats.copy(source.getSupportedImageFormats()); 
 
}

} // namespace environment
} // namespace openjaus

