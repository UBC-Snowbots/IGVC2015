/**
\file StillImageSensorConfigurationRecord.h

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
#include "openjaus/environment/Triggers/Fields/StillImageSensorConfigurationRecord.h"

namespace openjaus
{
namespace environment
{

StillImageSensorConfigurationRecord::StillImageSensorConfigurationRecord():
	sensorID(),
	frameSize(),
	stillImageFormat()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&frameSize);
	frameSize.setName("FrameSize");
	frameSize.setOptional(true);
	// Nothing to init

	fields.push_back(&stillImageFormat);
	stillImageFormat.setName("StillImageFormat");
	stillImageFormat.setOptional(true);
	// Nothing to init

}

StillImageSensorConfigurationRecord::StillImageSensorConfigurationRecord(const StillImageSensorConfigurationRecord &source)
{
	this->copy(const_cast<StillImageSensorConfigurationRecord&>(source));
}

StillImageSensorConfigurationRecord::~StillImageSensorConfigurationRecord()
{

}


uint16_t StillImageSensorConfigurationRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void StillImageSensorConfigurationRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

FrameSizeRefEnumeration::FrameSizeRefEnum StillImageSensorConfigurationRecord::getFrameSize(void)
{
	return this->frameSize.getValue();
}

void StillImageSensorConfigurationRecord::setFrameSize(FrameSizeRefEnumeration::FrameSizeRefEnum value)
{
	this->frameSize.setValue(value);
}

StillImageFormatEnumeration::StillImageFormatEnum StillImageSensorConfigurationRecord::getStillImageFormat(void)
{
	return this->stillImageFormat.getValue();
}

void StillImageSensorConfigurationRecord::setStillImageFormat(StillImageFormatEnumeration::StillImageFormatEnum value)
{
	this->stillImageFormat.setValue(value);
}

int StillImageSensorConfigurationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	if(this->isFrameSizeEnabled())
	{
		byteSize += dst->pack(frameSize);
	}
	if(this->isStillImageFormatEnabled())
	{
		byteSize += dst->pack(stillImageFormat);
	}
	return byteSize;
}
int StillImageSensorConfigurationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	if(this->isFrameSizeEnabled())
	{
		byteSize += src->unpack(frameSize);
	}
	if(this->isStillImageFormatEnabled())
	{
		byteSize += src->unpack(stillImageFormat);
	}
	return byteSize;
}

int StillImageSensorConfigurationRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += sensorID.length(); // sensorID
	if(this->isFrameSizeEnabled())
	{
		length += frameSize.length(); // frameSize
	}
	if(this->isStillImageFormatEnabled())
	{
		length += stillImageFormat.length(); // stillImageFormat
	}
	return length;
}

std::string StillImageSensorConfigurationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"StillImageSensorConfigurationRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFrameSizeEnabled value=\"" << std::boolalpha << this->isFrameSizeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isStillImageFormatEnabled value=\"" << std::boolalpha << this->isStillImageFormatEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	if(this->isFrameSizeEnabled())
	{
		oss << frameSize.toXml(level+1); // frameSize
	}
	if(this->isStillImageFormatEnabled())
	{
		oss << stillImageFormat.toXml(level+1); // stillImageFormat
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void StillImageSensorConfigurationRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t StillImageSensorConfigurationRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool StillImageSensorConfigurationRecord::isFrameSizeEnabled(void) const
{
	return (this->presenceVector & (0x01 << StillImageSensorConfigurationRecord::FRAMESIZE));
}

void StillImageSensorConfigurationRecord::enableFrameSize(void)
{
	this->presenceVector |= 0x01 << StillImageSensorConfigurationRecord::FRAMESIZE;
}

void StillImageSensorConfigurationRecord::disableFrameSize(void)
{
	this->presenceVector &= ~(0x01 << StillImageSensorConfigurationRecord::FRAMESIZE);
}

bool StillImageSensorConfigurationRecord::isStillImageFormatEnabled(void) const
{
	return (this->presenceVector & (0x01 << StillImageSensorConfigurationRecord::STILLIMAGEFORMAT));
}

void StillImageSensorConfigurationRecord::enableStillImageFormat(void)
{
	this->presenceVector |= 0x01 << StillImageSensorConfigurationRecord::STILLIMAGEFORMAT;
}

void StillImageSensorConfigurationRecord::disableStillImageFormat(void)
{
	this->presenceVector &= ~(0x01 << StillImageSensorConfigurationRecord::STILLIMAGEFORMAT);
}


void StillImageSensorConfigurationRecord::copy(StillImageSensorConfigurationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->frameSize.setName("FrameSizeRef");
	this->frameSize.setOptional(false);
	this->frameSize.setValue(source.getFrameSize()); 
 
	this->stillImageFormat.setName("StillImageFormat");
	this->stillImageFormat.setOptional(true);
	this->stillImageFormat.setValue(source.getStillImageFormat()); 
 
}

} // namespace environment
} // namespace openjaus

