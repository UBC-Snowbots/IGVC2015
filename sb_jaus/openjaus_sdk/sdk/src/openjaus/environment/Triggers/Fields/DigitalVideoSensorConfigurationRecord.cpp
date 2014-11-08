/**
\file DigitalVideoSensorConfigurationRecord.h

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
#include "openjaus/environment/Triggers/Fields/DigitalVideoSensorConfigurationRecord.h"

namespace openjaus
{
namespace environment
{

DigitalVideoSensorConfigurationRecord::DigitalVideoSensorConfigurationRecord():
	sensorID(),
	minimumBitRate(),
	maximumBitRate(),
	frameRate(),
	frameSize(),
	digitalFormat()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&minimumBitRate);
	minimumBitRate.setName("MinimumBitRate");
	minimumBitRate.setOptional(true);
	minimumBitRate.setValue(0);

	fields.push_back(&maximumBitRate);
	maximumBitRate.setName("MaximumBitRate");
	maximumBitRate.setOptional(true);
	maximumBitRate.setValue(0);

	fields.push_back(&frameRate);
	frameRate.setName("FrameRate");
	frameRate.setOptional(true);
	frameRate.setValue(0);

	fields.push_back(&frameSize);
	frameSize.setName("FrameSize");
	frameSize.setOptional(true);
	// Nothing to init

	fields.push_back(&digitalFormat);
	digitalFormat.setName("DigitalFormat");
	digitalFormat.setOptional(true);
	// Nothing to init

}

DigitalVideoSensorConfigurationRecord::DigitalVideoSensorConfigurationRecord(const DigitalVideoSensorConfigurationRecord &source)
{
	this->copy(const_cast<DigitalVideoSensorConfigurationRecord&>(source));
}

DigitalVideoSensorConfigurationRecord::~DigitalVideoSensorConfigurationRecord()
{

}


uint16_t DigitalVideoSensorConfigurationRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void DigitalVideoSensorConfigurationRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

uint16_t DigitalVideoSensorConfigurationRecord::getMinimumBitRate(void)
{
	return this->minimumBitRate.getValue();
}

void DigitalVideoSensorConfigurationRecord::setMinimumBitRate(uint16_t value)
{
	this->minimumBitRate.setValue(value);
}

uint16_t DigitalVideoSensorConfigurationRecord::getMaximumBitRate(void)
{
	return this->maximumBitRate.getValue();
}

void DigitalVideoSensorConfigurationRecord::setMaximumBitRate(uint16_t value)
{
	this->maximumBitRate.setValue(value);
}

uint8_t DigitalVideoSensorConfigurationRecord::getFrameRate(void)
{
	return this->frameRate.getValue();
}

void DigitalVideoSensorConfigurationRecord::setFrameRate(uint8_t value)
{
	this->frameRate.setValue(value);
}

FrameSizeRefEnumeration::FrameSizeRefEnum DigitalVideoSensorConfigurationRecord::getFrameSize(void)
{
	return this->frameSize.getValue();
}

void DigitalVideoSensorConfigurationRecord::setFrameSize(FrameSizeRefEnumeration::FrameSizeRefEnum value)
{
	this->frameSize.setValue(value);
}

DigitalFormatEnumeration::DigitalFormatEnum DigitalVideoSensorConfigurationRecord::getDigitalFormat(void)
{
	return this->digitalFormat.getValue();
}

void DigitalVideoSensorConfigurationRecord::setDigitalFormat(DigitalFormatEnumeration::DigitalFormatEnum value)
{
	this->digitalFormat.setValue(value);
}

int DigitalVideoSensorConfigurationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	if(this->isMinimumBitRateEnabled())
	{
		byteSize += dst->pack(minimumBitRate);
	}
	if(this->isMaximumBitRateEnabled())
	{
		byteSize += dst->pack(maximumBitRate);
	}
	if(this->isFrameRateEnabled())
	{
		byteSize += dst->pack(frameRate);
	}
	if(this->isFrameSizeEnabled())
	{
		byteSize += dst->pack(frameSize);
	}
	if(this->isDigitalFormatEnabled())
	{
		byteSize += dst->pack(digitalFormat);
	}
	return byteSize;
}
int DigitalVideoSensorConfigurationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	if(this->isMinimumBitRateEnabled())
	{
		byteSize += src->unpack(minimumBitRate);
	}
	if(this->isMaximumBitRateEnabled())
	{
		byteSize += src->unpack(maximumBitRate);
	}
	if(this->isFrameRateEnabled())
	{
		byteSize += src->unpack(frameRate);
	}
	if(this->isFrameSizeEnabled())
	{
		byteSize += src->unpack(frameSize);
	}
	if(this->isDigitalFormatEnabled())
	{
		byteSize += src->unpack(digitalFormat);
	}
	return byteSize;
}

int DigitalVideoSensorConfigurationRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += sensorID.length(); // sensorID
	if(this->isMinimumBitRateEnabled())
	{
		length += minimumBitRate.length(); // minimumBitRate
	}
	if(this->isMaximumBitRateEnabled())
	{
		length += maximumBitRate.length(); // maximumBitRate
	}
	if(this->isFrameRateEnabled())
	{
		length += frameRate.length(); // frameRate
	}
	if(this->isFrameSizeEnabled())
	{
		length += frameSize.length(); // frameSize
	}
	if(this->isDigitalFormatEnabled())
	{
		length += digitalFormat.length(); // digitalFormat
	}
	return length;
}

std::string DigitalVideoSensorConfigurationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"DigitalVideoSensorConfigurationRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumBitRateEnabled value=\"" << std::boolalpha << this->isMinimumBitRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumBitRateEnabled value=\"" << std::boolalpha << this->isMaximumBitRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFrameRateEnabled value=\"" << std::boolalpha << this->isFrameRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isFrameSizeEnabled value=\"" << std::boolalpha << this->isFrameSizeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isDigitalFormatEnabled value=\"" << std::boolalpha << this->isDigitalFormatEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	if(this->isMinimumBitRateEnabled())
	{
		oss << minimumBitRate.toXml(level+1); // minimumBitRate
	}
	if(this->isMaximumBitRateEnabled())
	{
		oss << maximumBitRate.toXml(level+1); // maximumBitRate
	}
	if(this->isFrameRateEnabled())
	{
		oss << frameRate.toXml(level+1); // frameRate
	}
	if(this->isFrameSizeEnabled())
	{
		oss << frameSize.toXml(level+1); // frameSize
	}
	if(this->isDigitalFormatEnabled())
	{
		oss << digitalFormat.toXml(level+1); // digitalFormat
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void DigitalVideoSensorConfigurationRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t DigitalVideoSensorConfigurationRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool DigitalVideoSensorConfigurationRecord::isMinimumBitRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorConfigurationRecord::MINIMUMBITRATE));
}

void DigitalVideoSensorConfigurationRecord::enableMinimumBitRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorConfigurationRecord::MINIMUMBITRATE;
}

void DigitalVideoSensorConfigurationRecord::disableMinimumBitRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorConfigurationRecord::MINIMUMBITRATE);
}

bool DigitalVideoSensorConfigurationRecord::isMaximumBitRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorConfigurationRecord::MAXIMUMBITRATE));
}

void DigitalVideoSensorConfigurationRecord::enableMaximumBitRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorConfigurationRecord::MAXIMUMBITRATE;
}

void DigitalVideoSensorConfigurationRecord::disableMaximumBitRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorConfigurationRecord::MAXIMUMBITRATE);
}

bool DigitalVideoSensorConfigurationRecord::isFrameRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorConfigurationRecord::FRAMERATE));
}

void DigitalVideoSensorConfigurationRecord::enableFrameRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorConfigurationRecord::FRAMERATE;
}

void DigitalVideoSensorConfigurationRecord::disableFrameRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorConfigurationRecord::FRAMERATE);
}

bool DigitalVideoSensorConfigurationRecord::isFrameSizeEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorConfigurationRecord::FRAMESIZE));
}

void DigitalVideoSensorConfigurationRecord::enableFrameSize(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorConfigurationRecord::FRAMESIZE;
}

void DigitalVideoSensorConfigurationRecord::disableFrameSize(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorConfigurationRecord::FRAMESIZE);
}

bool DigitalVideoSensorConfigurationRecord::isDigitalFormatEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorConfigurationRecord::DIGITALFORMAT));
}

void DigitalVideoSensorConfigurationRecord::enableDigitalFormat(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorConfigurationRecord::DIGITALFORMAT;
}

void DigitalVideoSensorConfigurationRecord::disableDigitalFormat(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorConfigurationRecord::DIGITALFORMAT);
}


void DigitalVideoSensorConfigurationRecord::copy(DigitalVideoSensorConfigurationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->minimumBitRate.setName("MinimumBitRate");
	this->minimumBitRate.setOptional(true);
	this->minimumBitRate.setValue(source.getMinimumBitRate()); 
 
	this->maximumBitRate.setName("MaximumBitRate");
	this->maximumBitRate.setOptional(true);
	this->maximumBitRate.setValue(source.getMaximumBitRate()); 
 
	this->frameRate.setName("FrameRate");
	this->frameRate.setOptional(true);
	this->frameRate.setValue(source.getFrameRate()); 
 
	this->frameSize.setName("FrameSizeRef");
	this->frameSize.setOptional(false);
	this->frameSize.setValue(source.getFrameSize()); 
 
	this->digitalFormat.setName("DigitalFormat");
	this->digitalFormat.setOptional(true);
	this->digitalFormat.setValue(source.getDigitalFormat()); 
 
}

} // namespace environment
} // namespace openjaus

