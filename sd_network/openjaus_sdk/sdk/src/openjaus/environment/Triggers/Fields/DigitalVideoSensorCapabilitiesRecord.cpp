/**
\file DigitalVideoSensorCapabilitiesRecord.h

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
#include "openjaus/environment/Triggers/Fields/DigitalVideoSensorCapabilitiesRecord.h"

namespace openjaus
{
namespace environment
{

DigitalVideoSensorCapabilitiesRecord::DigitalVideoSensorCapabilitiesRecord():
	sensorID(),
	minimumBitRate(),
	maximumBitRate(),
	minimumFrameRate(),
	maximumFrameRate(),
	supportedFrameSizes(),
	supportedDigitalFormats()
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

	fields.push_back(&minimumFrameRate);
	minimumFrameRate.setName("MinimumFrameRate");
	minimumFrameRate.setOptional(true);
	minimumFrameRate.setValue(0);

	fields.push_back(&maximumFrameRate);
	maximumFrameRate.setName("MaximumFrameRate");
	maximumFrameRate.setOptional(true);
	maximumFrameRate.setValue(0);

	fields.push_back(&supportedFrameSizes);
	supportedFrameSizes.setName("SupportedFrameSizes");
	supportedFrameSizes.setOptional(true);
	// Nothing

	fields.push_back(&supportedDigitalFormats);
	supportedDigitalFormats.setName("SupportedDigitalFormats");
	supportedDigitalFormats.setOptional(true);
	// Nothing

}

DigitalVideoSensorCapabilitiesRecord::DigitalVideoSensorCapabilitiesRecord(const DigitalVideoSensorCapabilitiesRecord &source)
{
	this->copy(const_cast<DigitalVideoSensorCapabilitiesRecord&>(source));
}

DigitalVideoSensorCapabilitiesRecord::~DigitalVideoSensorCapabilitiesRecord()
{

}


uint16_t DigitalVideoSensorCapabilitiesRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void DigitalVideoSensorCapabilitiesRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

uint16_t DigitalVideoSensorCapabilitiesRecord::getMinimumBitRate(void)
{
	return this->minimumBitRate.getValue();
}

void DigitalVideoSensorCapabilitiesRecord::setMinimumBitRate(uint16_t value)
{
	this->minimumBitRate.setValue(value);
}

uint16_t DigitalVideoSensorCapabilitiesRecord::getMaximumBitRate(void)
{
	return this->maximumBitRate.getValue();
}

void DigitalVideoSensorCapabilitiesRecord::setMaximumBitRate(uint16_t value)
{
	this->maximumBitRate.setValue(value);
}

uint8_t DigitalVideoSensorCapabilitiesRecord::getMinimumFrameRate(void)
{
	return this->minimumFrameRate.getValue();
}

void DigitalVideoSensorCapabilitiesRecord::setMinimumFrameRate(uint8_t value)
{
	this->minimumFrameRate.setValue(value);
}

uint8_t DigitalVideoSensorCapabilitiesRecord::getMaximumFrameRate(void)
{
	return this->maximumFrameRate.getValue();
}

void DigitalVideoSensorCapabilitiesRecord::setMaximumFrameRate(uint8_t value)
{
	this->maximumFrameRate.setValue(value);
}

SupportedFrameSizesBitField& DigitalVideoSensorCapabilitiesRecord::getSupportedFrameSizes(void)
{
	return this->supportedFrameSizes;
}

SupportedDigitalFormatsBitField& DigitalVideoSensorCapabilitiesRecord::getSupportedDigitalFormats(void)
{
	return this->supportedDigitalFormats;
}

int DigitalVideoSensorCapabilitiesRecord::to(system::Buffer *dst)
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
	if(this->isMinimumFrameRateEnabled())
	{
		byteSize += dst->pack(minimumFrameRate);
	}
	if(this->isMaximumFrameRateEnabled())
	{
		byteSize += dst->pack(maximumFrameRate);
	}
	if(this->isSupportedFrameSizesEnabled())
	{
		byteSize += dst->pack(supportedFrameSizes);
	}
	if(this->isSupportedDigitalFormatsEnabled())
	{
		byteSize += dst->pack(supportedDigitalFormats);
	}
	return byteSize;
}
int DigitalVideoSensorCapabilitiesRecord::from(system::Buffer *src)
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
	if(this->isMinimumFrameRateEnabled())
	{
		byteSize += src->unpack(minimumFrameRate);
	}
	if(this->isMaximumFrameRateEnabled())
	{
		byteSize += src->unpack(maximumFrameRate);
	}
	if(this->isSupportedFrameSizesEnabled())
	{
		byteSize += src->unpack(supportedFrameSizes);
	}
	if(this->isSupportedDigitalFormatsEnabled())
	{
		byteSize += src->unpack(supportedDigitalFormats);
	}
	return byteSize;
}

int DigitalVideoSensorCapabilitiesRecord::length(void)
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
	if(this->isMinimumFrameRateEnabled())
	{
		length += minimumFrameRate.length(); // minimumFrameRate
	}
	if(this->isMaximumFrameRateEnabled())
	{
		length += maximumFrameRate.length(); // maximumFrameRate
	}
	if(this->isSupportedFrameSizesEnabled())
	{
		length += supportedFrameSizes.length(); // supportedFrameSizes
	}
	if(this->isSupportedDigitalFormatsEnabled())
	{
		length += supportedDigitalFormats.length(); // supportedDigitalFormats
	}
	return length;
}

std::string DigitalVideoSensorCapabilitiesRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"DigitalVideoSensorCapabilitiesRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumBitRateEnabled value=\"" << std::boolalpha << this->isMinimumBitRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumBitRateEnabled value=\"" << std::boolalpha << this->isMaximumBitRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumFrameRateEnabled value=\"" << std::boolalpha << this->isMinimumFrameRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumFrameRateEnabled value=\"" << std::boolalpha << this->isMaximumFrameRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedFrameSizesEnabled value=\"" << std::boolalpha << this->isSupportedFrameSizesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedDigitalFormatsEnabled value=\"" << std::boolalpha << this->isSupportedDigitalFormatsEnabled() << "\"/>\n";
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
	if(this->isMinimumFrameRateEnabled())
	{
		oss << minimumFrameRate.toXml(level+1); // minimumFrameRate
	}
	if(this->isMaximumFrameRateEnabled())
	{
		oss << maximumFrameRate.toXml(level+1); // maximumFrameRate
	}
	if(this->isSupportedFrameSizesEnabled())
	{
		oss << supportedFrameSizes.toXml(level+1); // supportedFrameSizes
	}
	if(this->isSupportedDigitalFormatsEnabled())
	{
		oss << supportedDigitalFormats.toXml(level+1); // supportedDigitalFormats
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void DigitalVideoSensorCapabilitiesRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t DigitalVideoSensorCapabilitiesRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool DigitalVideoSensorCapabilitiesRecord::isMinimumBitRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorCapabilitiesRecord::MINIMUMBITRATE));
}

void DigitalVideoSensorCapabilitiesRecord::enableMinimumBitRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorCapabilitiesRecord::MINIMUMBITRATE;
}

void DigitalVideoSensorCapabilitiesRecord::disableMinimumBitRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorCapabilitiesRecord::MINIMUMBITRATE);
}

bool DigitalVideoSensorCapabilitiesRecord::isMaximumBitRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorCapabilitiesRecord::MAXIMUMBITRATE));
}

void DigitalVideoSensorCapabilitiesRecord::enableMaximumBitRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorCapabilitiesRecord::MAXIMUMBITRATE;
}

void DigitalVideoSensorCapabilitiesRecord::disableMaximumBitRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorCapabilitiesRecord::MAXIMUMBITRATE);
}

bool DigitalVideoSensorCapabilitiesRecord::isMinimumFrameRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorCapabilitiesRecord::MINIMUMFRAMERATE));
}

void DigitalVideoSensorCapabilitiesRecord::enableMinimumFrameRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorCapabilitiesRecord::MINIMUMFRAMERATE;
}

void DigitalVideoSensorCapabilitiesRecord::disableMinimumFrameRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorCapabilitiesRecord::MINIMUMFRAMERATE);
}

bool DigitalVideoSensorCapabilitiesRecord::isMaximumFrameRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorCapabilitiesRecord::MAXIMUMFRAMERATE));
}

void DigitalVideoSensorCapabilitiesRecord::enableMaximumFrameRate(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorCapabilitiesRecord::MAXIMUMFRAMERATE;
}

void DigitalVideoSensorCapabilitiesRecord::disableMaximumFrameRate(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorCapabilitiesRecord::MAXIMUMFRAMERATE);
}

bool DigitalVideoSensorCapabilitiesRecord::isSupportedFrameSizesEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorCapabilitiesRecord::SUPPORTEDFRAMESIZES));
}

void DigitalVideoSensorCapabilitiesRecord::enableSupportedFrameSizes(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorCapabilitiesRecord::SUPPORTEDFRAMESIZES;
}

void DigitalVideoSensorCapabilitiesRecord::disableSupportedFrameSizes(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorCapabilitiesRecord::SUPPORTEDFRAMESIZES);
}

bool DigitalVideoSensorCapabilitiesRecord::isSupportedDigitalFormatsEnabled(void) const
{
	return (this->presenceVector & (0x01 << DigitalVideoSensorCapabilitiesRecord::SUPPORTEDDIGITALFORMATS));
}

void DigitalVideoSensorCapabilitiesRecord::enableSupportedDigitalFormats(void)
{
	this->presenceVector |= 0x01 << DigitalVideoSensorCapabilitiesRecord::SUPPORTEDDIGITALFORMATS;
}

void DigitalVideoSensorCapabilitiesRecord::disableSupportedDigitalFormats(void)
{
	this->presenceVector &= ~(0x01 << DigitalVideoSensorCapabilitiesRecord::SUPPORTEDDIGITALFORMATS);
}


void DigitalVideoSensorCapabilitiesRecord::copy(DigitalVideoSensorCapabilitiesRecord& source)
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
 
	this->minimumFrameRate.setName("MinimumFrameRate");
	this->minimumFrameRate.setOptional(true);
	this->minimumFrameRate.setValue(source.getMinimumFrameRate()); 
 
	this->maximumFrameRate.setName("MaximumFrameRate");
	this->maximumFrameRate.setOptional(true);
	this->maximumFrameRate.setValue(source.getMaximumFrameRate()); 
 
	this->supportedFrameSizes.copy(source.getSupportedFrameSizes()); 
 
	this->supportedDigitalFormats.copy(source.getSupportedDigitalFormats()); 
 
}

} // namespace environment
} // namespace openjaus

