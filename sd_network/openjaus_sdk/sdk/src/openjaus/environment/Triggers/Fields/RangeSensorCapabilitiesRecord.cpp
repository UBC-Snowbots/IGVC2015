/**
\file RangeSensorCapabilitiesRecord.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorCapabilitiesRecord.h"

namespace openjaus
{
namespace environment
{

RangeSensorCapabilitiesRecord::RangeSensorCapabilitiesRecord():
	sensorID(),
	sensorName(),
	supportedStates(),
	minimumHorizontalFieldOfViewStartAngle_rad(),
	maximumHorizontalFieldOfViewStopAngle_rad(),
	minimumVerticalFieldOfViewStartAngle_rad(),
	maximumVerticalFieldOfViewStopAngle_rad(),
	miniumumUpdateRate_Hz(),
	maximumUpdateRate_Hz(),
	minimumRange_m(),
	maximumRange_m(),
	supportedCompression(),
	coordinateTransformationSupported()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&sensorName);
	sensorName.setName("SensorName");
	sensorName.setOptional(false);
	sensorName.setSizeType(model::fields::UNSIGNED_BYTE);

	fields.push_back(&supportedStates);
	supportedStates.setName("SupportedStates");
	supportedStates.setOptional(true);
	// Nothing

	fields.push_back(&minimumHorizontalFieldOfViewStartAngle_rad);
	minimumHorizontalFieldOfViewStartAngle_rad.setName("MinimumHorizontalFieldOfViewStartAngle");
	minimumHorizontalFieldOfViewStartAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumHorizontalFieldOfViewStopAngle_rad);
	maximumHorizontalFieldOfViewStopAngle_rad.setName("MaximumHorizontalFieldOfViewStopAngle");
	maximumHorizontalFieldOfViewStopAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&minimumVerticalFieldOfViewStartAngle_rad);
	minimumVerticalFieldOfViewStartAngle_rad.setName("MinimumVerticalFieldOfViewStartAngle");
	minimumVerticalFieldOfViewStartAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumVerticalFieldOfViewStopAngle_rad);
	maximumVerticalFieldOfViewStopAngle_rad.setName("MaximumVerticalFieldOfViewStopAngle");
	maximumVerticalFieldOfViewStopAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&miniumumUpdateRate_Hz);
	miniumumUpdateRate_Hz.setName("MiniumumUpdateRate");
	miniumumUpdateRate_Hz.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumUpdateRate_Hz);
	maximumUpdateRate_Hz.setName("MaximumUpdateRate");
	maximumUpdateRate_Hz.setOptional(true);
	// Nothing to init

	fields.push_back(&minimumRange_m);
	minimumRange_m.setName("MinimumRange");
	minimumRange_m.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumRange_m);
	maximumRange_m.setName("MaximumRange");
	maximumRange_m.setOptional(true);
	// Nothing to init

	fields.push_back(&supportedCompression);
	supportedCompression.setName("SupportedCompression");
	supportedCompression.setOptional(true);
	// Nothing

	fields.push_back(&coordinateTransformationSupported);
	coordinateTransformationSupported.setName("CoordinateTransformationSupported");
	coordinateTransformationSupported.setOptional(true);
	// Nothing to init

}

RangeSensorCapabilitiesRecord::RangeSensorCapabilitiesRecord(const RangeSensorCapabilitiesRecord &source)
{
	this->copy(const_cast<RangeSensorCapabilitiesRecord&>(source));
}

RangeSensorCapabilitiesRecord::~RangeSensorCapabilitiesRecord()
{

}


uint16_t RangeSensorCapabilitiesRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void RangeSensorCapabilitiesRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

std::string RangeSensorCapabilitiesRecord::getSensorName(void)
{
	return this->sensorName.getValue();
}

void RangeSensorCapabilitiesRecord::setSensorName(std::string value)
{
	this->sensorName.setValue(value);
}

SupportedStatesBitField& RangeSensorCapabilitiesRecord::getSupportedStates(void)
{
	return this->supportedStates;
}

double RangeSensorCapabilitiesRecord::getMinimumHorizontalFieldOfViewStartAngle_rad(void)
{
	return this->minimumHorizontalFieldOfViewStartAngle_rad.getValue();
}

void RangeSensorCapabilitiesRecord::setMinimumHorizontalFieldOfViewStartAngle_rad(double value)
{
	this->minimumHorizontalFieldOfViewStartAngle_rad.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMaximumHorizontalFieldOfViewStopAngle_rad(void)
{
	return this->maximumHorizontalFieldOfViewStopAngle_rad.getValue();
}

void RangeSensorCapabilitiesRecord::setMaximumHorizontalFieldOfViewStopAngle_rad(double value)
{
	this->maximumHorizontalFieldOfViewStopAngle_rad.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMinimumVerticalFieldOfViewStartAngle_rad(void)
{
	return this->minimumVerticalFieldOfViewStartAngle_rad.getValue();
}

void RangeSensorCapabilitiesRecord::setMinimumVerticalFieldOfViewStartAngle_rad(double value)
{
	this->minimumVerticalFieldOfViewStartAngle_rad.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMaximumVerticalFieldOfViewStopAngle_rad(void)
{
	return this->maximumVerticalFieldOfViewStopAngle_rad.getValue();
}

void RangeSensorCapabilitiesRecord::setMaximumVerticalFieldOfViewStopAngle_rad(double value)
{
	this->maximumVerticalFieldOfViewStopAngle_rad.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMiniumumUpdateRate_Hz(void)
{
	return this->miniumumUpdateRate_Hz.getValue();
}

void RangeSensorCapabilitiesRecord::setMiniumumUpdateRate_Hz(double value)
{
	this->miniumumUpdateRate_Hz.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMaximumUpdateRate_Hz(void)
{
	return this->maximumUpdateRate_Hz.getValue();
}

void RangeSensorCapabilitiesRecord::setMaximumUpdateRate_Hz(double value)
{
	this->maximumUpdateRate_Hz.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMinimumRange_m(void)
{
	return this->minimumRange_m.getValue();
}

void RangeSensorCapabilitiesRecord::setMinimumRange_m(double value)
{
	this->minimumRange_m.setValue(value);
}

double RangeSensorCapabilitiesRecord::getMaximumRange_m(void)
{
	return this->maximumRange_m.getValue();
}

void RangeSensorCapabilitiesRecord::setMaximumRange_m(double value)
{
	this->maximumRange_m.setValue(value);
}

SupportedCompressionBitField& RangeSensorCapabilitiesRecord::getSupportedCompression(void)
{
	return this->supportedCompression;
}

CoordinateTransformationSupportedEnumeration::CoordinateTransformationSupportedEnum RangeSensorCapabilitiesRecord::getCoordinateTransformationSupported(void)
{
	return this->coordinateTransformationSupported.getValue();
}

void RangeSensorCapabilitiesRecord::setCoordinateTransformationSupported(CoordinateTransformationSupportedEnumeration::CoordinateTransformationSupportedEnum value)
{
	this->coordinateTransformationSupported.setValue(value);
}

int RangeSensorCapabilitiesRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	byteSize += dst->pack(sensorName);
	if(this->isSupportedStatesEnabled())
	{
		byteSize += dst->pack(supportedStates);
	}
	if(this->isMinimumHorizontalFieldOfViewStartAngleEnabled())
	{
		byteSize += dst->pack(minimumHorizontalFieldOfViewStartAngle_rad);
	}
	if(this->isMaximumHorizontalFieldOfViewStopAngleEnabled())
	{
		byteSize += dst->pack(maximumHorizontalFieldOfViewStopAngle_rad);
	}
	if(this->isMinimumVerticalFieldOfViewStartAngleEnabled())
	{
		byteSize += dst->pack(minimumVerticalFieldOfViewStartAngle_rad);
	}
	if(this->isMaximumVerticalFieldOfViewStopAngleEnabled())
	{
		byteSize += dst->pack(maximumVerticalFieldOfViewStopAngle_rad);
	}
	if(this->isMiniumumUpdateRateEnabled())
	{
		byteSize += dst->pack(miniumumUpdateRate_Hz);
	}
	if(this->isMaximumUpdateRateEnabled())
	{
		byteSize += dst->pack(maximumUpdateRate_Hz);
	}
	if(this->isMinimumRangeEnabled())
	{
		byteSize += dst->pack(minimumRange_m);
	}
	if(this->isMaximumRangeEnabled())
	{
		byteSize += dst->pack(maximumRange_m);
	}
	if(this->isSupportedCompressionEnabled())
	{
		byteSize += dst->pack(supportedCompression);
	}
	if(this->isCoordinateTransformationSupportedEnabled())
	{
		byteSize += dst->pack(coordinateTransformationSupported);
	}
	return byteSize;
}
int RangeSensorCapabilitiesRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	byteSize += src->unpack(sensorName);
	if(this->isSupportedStatesEnabled())
	{
		byteSize += src->unpack(supportedStates);
	}
	if(this->isMinimumHorizontalFieldOfViewStartAngleEnabled())
	{
		byteSize += src->unpack(minimumHorizontalFieldOfViewStartAngle_rad);
	}
	if(this->isMaximumHorizontalFieldOfViewStopAngleEnabled())
	{
		byteSize += src->unpack(maximumHorizontalFieldOfViewStopAngle_rad);
	}
	if(this->isMinimumVerticalFieldOfViewStartAngleEnabled())
	{
		byteSize += src->unpack(minimumVerticalFieldOfViewStartAngle_rad);
	}
	if(this->isMaximumVerticalFieldOfViewStopAngleEnabled())
	{
		byteSize += src->unpack(maximumVerticalFieldOfViewStopAngle_rad);
	}
	if(this->isMiniumumUpdateRateEnabled())
	{
		byteSize += src->unpack(miniumumUpdateRate_Hz);
	}
	if(this->isMaximumUpdateRateEnabled())
	{
		byteSize += src->unpack(maximumUpdateRate_Hz);
	}
	if(this->isMinimumRangeEnabled())
	{
		byteSize += src->unpack(minimumRange_m);
	}
	if(this->isMaximumRangeEnabled())
	{
		byteSize += src->unpack(maximumRange_m);
	}
	if(this->isSupportedCompressionEnabled())
	{
		byteSize += src->unpack(supportedCompression);
	}
	if(this->isCoordinateTransformationSupportedEnabled())
	{
		byteSize += src->unpack(coordinateTransformationSupported);
	}
	return byteSize;
}

int RangeSensorCapabilitiesRecord::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // PresenceVector
	length += sensorID.length(); // sensorID
	length += sensorName.length(); // sensorName
	if(this->isSupportedStatesEnabled())
	{
		length += supportedStates.length(); // supportedStates
	}
	if(this->isMinimumHorizontalFieldOfViewStartAngleEnabled())
	{
		length += minimumHorizontalFieldOfViewStartAngle_rad.length(); // minimumHorizontalFieldOfViewStartAngle_rad
	}
	if(this->isMaximumHorizontalFieldOfViewStopAngleEnabled())
	{
		length += maximumHorizontalFieldOfViewStopAngle_rad.length(); // maximumHorizontalFieldOfViewStopAngle_rad
	}
	if(this->isMinimumVerticalFieldOfViewStartAngleEnabled())
	{
		length += minimumVerticalFieldOfViewStartAngle_rad.length(); // minimumVerticalFieldOfViewStartAngle_rad
	}
	if(this->isMaximumVerticalFieldOfViewStopAngleEnabled())
	{
		length += maximumVerticalFieldOfViewStopAngle_rad.length(); // maximumVerticalFieldOfViewStopAngle_rad
	}
	if(this->isMiniumumUpdateRateEnabled())
	{
		length += miniumumUpdateRate_Hz.length(); // miniumumUpdateRate_Hz
	}
	if(this->isMaximumUpdateRateEnabled())
	{
		length += maximumUpdateRate_Hz.length(); // maximumUpdateRate_Hz
	}
	if(this->isMinimumRangeEnabled())
	{
		length += minimumRange_m.length(); // minimumRange_m
	}
	if(this->isMaximumRangeEnabled())
	{
		length += maximumRange_m.length(); // maximumRange_m
	}
	if(this->isSupportedCompressionEnabled())
	{
		length += supportedCompression.length(); // supportedCompression
	}
	if(this->isCoordinateTransformationSupportedEnabled())
	{
		length += coordinateTransformationSupported.length(); // coordinateTransformationSupported
	}
	return length;
}

std::string RangeSensorCapabilitiesRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RangeSensorCapabilitiesRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedStatesEnabled value=\"" << std::boolalpha << this->isSupportedStatesEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumHorizontalFieldOfViewStartAngleEnabled value=\"" << std::boolalpha << this->isMinimumHorizontalFieldOfViewStartAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumHorizontalFieldOfViewStopAngleEnabled value=\"" << std::boolalpha << this->isMaximumHorizontalFieldOfViewStopAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumVerticalFieldOfViewStartAngleEnabled value=\"" << std::boolalpha << this->isMinimumVerticalFieldOfViewStartAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumVerticalFieldOfViewStopAngleEnabled value=\"" << std::boolalpha << this->isMaximumVerticalFieldOfViewStopAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMiniumumUpdateRateEnabled value=\"" << std::boolalpha << this->isMiniumumUpdateRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumUpdateRateEnabled value=\"" << std::boolalpha << this->isMaximumUpdateRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumRangeEnabled value=\"" << std::boolalpha << this->isMinimumRangeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumRangeEnabled value=\"" << std::boolalpha << this->isMaximumRangeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSupportedCompressionEnabled value=\"" << std::boolalpha << this->isSupportedCompressionEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isCoordinateTransformationSupportedEnabled value=\"" << std::boolalpha << this->isCoordinateTransformationSupportedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	oss << sensorName.toXml(level+1); // sensorName
	if(this->isSupportedStatesEnabled())
	{
		oss << supportedStates.toXml(level+1); // supportedStates
	}
	if(this->isMinimumHorizontalFieldOfViewStartAngleEnabled())
	{
		oss << minimumHorizontalFieldOfViewStartAngle_rad.toXml(level+1); // minimumHorizontalFieldOfViewStartAngle_rad
	}
	if(this->isMaximumHorizontalFieldOfViewStopAngleEnabled())
	{
		oss << maximumHorizontalFieldOfViewStopAngle_rad.toXml(level+1); // maximumHorizontalFieldOfViewStopAngle_rad
	}
	if(this->isMinimumVerticalFieldOfViewStartAngleEnabled())
	{
		oss << minimumVerticalFieldOfViewStartAngle_rad.toXml(level+1); // minimumVerticalFieldOfViewStartAngle_rad
	}
	if(this->isMaximumVerticalFieldOfViewStopAngleEnabled())
	{
		oss << maximumVerticalFieldOfViewStopAngle_rad.toXml(level+1); // maximumVerticalFieldOfViewStopAngle_rad
	}
	if(this->isMiniumumUpdateRateEnabled())
	{
		oss << miniumumUpdateRate_Hz.toXml(level+1); // miniumumUpdateRate_Hz
	}
	if(this->isMaximumUpdateRateEnabled())
	{
		oss << maximumUpdateRate_Hz.toXml(level+1); // maximumUpdateRate_Hz
	}
	if(this->isMinimumRangeEnabled())
	{
		oss << minimumRange_m.toXml(level+1); // minimumRange_m
	}
	if(this->isMaximumRangeEnabled())
	{
		oss << maximumRange_m.toXml(level+1); // maximumRange_m
	}
	if(this->isSupportedCompressionEnabled())
	{
		oss << supportedCompression.toXml(level+1); // supportedCompression
	}
	if(this->isCoordinateTransformationSupportedEnabled())
	{
		oss << coordinateTransformationSupported.toXml(level+1); // coordinateTransformationSupported
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void RangeSensorCapabilitiesRecord::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t RangeSensorCapabilitiesRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool RangeSensorCapabilitiesRecord::isSupportedStatesEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::SUPPORTEDSTATES));
}

void RangeSensorCapabilitiesRecord::enableSupportedStates(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::SUPPORTEDSTATES;
}

void RangeSensorCapabilitiesRecord::disableSupportedStates(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::SUPPORTEDSTATES);
}

bool RangeSensorCapabilitiesRecord::isMinimumHorizontalFieldOfViewStartAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MINIMUMHORIZONTALFIELDOFVIEWSTARTANGLE_RAD));
}

void RangeSensorCapabilitiesRecord::enableMinimumHorizontalFieldOfViewStartAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MINIMUMHORIZONTALFIELDOFVIEWSTARTANGLE_RAD;
}

void RangeSensorCapabilitiesRecord::disableMinimumHorizontalFieldOfViewStartAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MINIMUMHORIZONTALFIELDOFVIEWSTARTANGLE_RAD);
}

bool RangeSensorCapabilitiesRecord::isMaximumHorizontalFieldOfViewStopAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MAXIMUMHORIZONTALFIELDOFVIEWSTOPANGLE_RAD));
}

void RangeSensorCapabilitiesRecord::enableMaximumHorizontalFieldOfViewStopAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MAXIMUMHORIZONTALFIELDOFVIEWSTOPANGLE_RAD;
}

void RangeSensorCapabilitiesRecord::disableMaximumHorizontalFieldOfViewStopAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MAXIMUMHORIZONTALFIELDOFVIEWSTOPANGLE_RAD);
}

bool RangeSensorCapabilitiesRecord::isMinimumVerticalFieldOfViewStartAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MINIMUMVERTICALFIELDOFVIEWSTARTANGLE_RAD));
}

void RangeSensorCapabilitiesRecord::enableMinimumVerticalFieldOfViewStartAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MINIMUMVERTICALFIELDOFVIEWSTARTANGLE_RAD;
}

void RangeSensorCapabilitiesRecord::disableMinimumVerticalFieldOfViewStartAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MINIMUMVERTICALFIELDOFVIEWSTARTANGLE_RAD);
}

bool RangeSensorCapabilitiesRecord::isMaximumVerticalFieldOfViewStopAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MAXIMUMVERTICALFIELDOFVIEWSTOPANGLE_RAD));
}

void RangeSensorCapabilitiesRecord::enableMaximumVerticalFieldOfViewStopAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MAXIMUMVERTICALFIELDOFVIEWSTOPANGLE_RAD;
}

void RangeSensorCapabilitiesRecord::disableMaximumVerticalFieldOfViewStopAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MAXIMUMVERTICALFIELDOFVIEWSTOPANGLE_RAD);
}

bool RangeSensorCapabilitiesRecord::isMiniumumUpdateRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MINIUMUMUPDATERATE_HZ));
}

void RangeSensorCapabilitiesRecord::enableMiniumumUpdateRate(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MINIUMUMUPDATERATE_HZ;
}

void RangeSensorCapabilitiesRecord::disableMiniumumUpdateRate(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MINIUMUMUPDATERATE_HZ);
}

bool RangeSensorCapabilitiesRecord::isMaximumUpdateRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MAXIMUMUPDATERATE_HZ));
}

void RangeSensorCapabilitiesRecord::enableMaximumUpdateRate(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MAXIMUMUPDATERATE_HZ;
}

void RangeSensorCapabilitiesRecord::disableMaximumUpdateRate(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MAXIMUMUPDATERATE_HZ);
}

bool RangeSensorCapabilitiesRecord::isMinimumRangeEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MINIMUMRANGE_M));
}

void RangeSensorCapabilitiesRecord::enableMinimumRange(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MINIMUMRANGE_M;
}

void RangeSensorCapabilitiesRecord::disableMinimumRange(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MINIMUMRANGE_M);
}

bool RangeSensorCapabilitiesRecord::isMaximumRangeEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::MAXIMUMRANGE_M));
}

void RangeSensorCapabilitiesRecord::enableMaximumRange(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::MAXIMUMRANGE_M;
}

void RangeSensorCapabilitiesRecord::disableMaximumRange(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::MAXIMUMRANGE_M);
}

bool RangeSensorCapabilitiesRecord::isSupportedCompressionEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::SUPPORTEDCOMPRESSION));
}

void RangeSensorCapabilitiesRecord::enableSupportedCompression(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::SUPPORTEDCOMPRESSION;
}

void RangeSensorCapabilitiesRecord::disableSupportedCompression(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::SUPPORTEDCOMPRESSION);
}

bool RangeSensorCapabilitiesRecord::isCoordinateTransformationSupportedEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorCapabilitiesRecord::COORDINATETRANSFORMATIONSUPPORTED));
}

void RangeSensorCapabilitiesRecord::enableCoordinateTransformationSupported(void)
{
	this->presenceVector |= 0x01 << RangeSensorCapabilitiesRecord::COORDINATETRANSFORMATIONSUPPORTED;
}

void RangeSensorCapabilitiesRecord::disableCoordinateTransformationSupported(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorCapabilitiesRecord::COORDINATETRANSFORMATIONSUPPORTED);
}


void RangeSensorCapabilitiesRecord::copy(RangeSensorCapabilitiesRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->sensorName.setName("SensorName");
	this->sensorName.setOptional(false);
	this->sensorName.setValue(source.getSensorName());
	this->sensorName.setSizeType(model::fields::UNSIGNED_BYTE); 
 
	this->supportedStates.copy(source.getSupportedStates()); 
 
	this->minimumHorizontalFieldOfViewStartAngle_rad.setName("MinimumHorizontalFieldOfViewStartAngle");
	this->minimumHorizontalFieldOfViewStartAngle_rad.setOptional(true);
	this->minimumHorizontalFieldOfViewStartAngle_rad.setValue(source.getMinimumHorizontalFieldOfViewStartAngle_rad()); 
 
	this->maximumHorizontalFieldOfViewStopAngle_rad.setName("MaximumHorizontalFieldOfViewStopAngle");
	this->maximumHorizontalFieldOfViewStopAngle_rad.setOptional(true);
	this->maximumHorizontalFieldOfViewStopAngle_rad.setValue(source.getMaximumHorizontalFieldOfViewStopAngle_rad()); 
 
	this->minimumVerticalFieldOfViewStartAngle_rad.setName("MinimumVerticalFieldOfViewStartAngle");
	this->minimumVerticalFieldOfViewStartAngle_rad.setOptional(true);
	this->minimumVerticalFieldOfViewStartAngle_rad.setValue(source.getMinimumVerticalFieldOfViewStartAngle_rad()); 
 
	this->maximumVerticalFieldOfViewStopAngle_rad.setName("MaximumVerticalFieldOfViewStopAngle");
	this->maximumVerticalFieldOfViewStopAngle_rad.setOptional(true);
	this->maximumVerticalFieldOfViewStopAngle_rad.setValue(source.getMaximumVerticalFieldOfViewStopAngle_rad()); 
 
	this->miniumumUpdateRate_Hz.setName("MiniumumUpdateRate");
	this->miniumumUpdateRate_Hz.setOptional(true);
	this->miniumumUpdateRate_Hz.setValue(source.getMiniumumUpdateRate_Hz()); 
 
	this->maximumUpdateRate_Hz.setName("MaximumUpdateRate");
	this->maximumUpdateRate_Hz.setOptional(true);
	this->maximumUpdateRate_Hz.setValue(source.getMaximumUpdateRate_Hz()); 
 
	this->minimumRange_m.setName("MinimumRange");
	this->minimumRange_m.setOptional(true);
	this->minimumRange_m.setValue(source.getMinimumRange_m()); 
 
	this->maximumRange_m.setName("MaximumRange");
	this->maximumRange_m.setOptional(true);
	this->maximumRange_m.setValue(source.getMaximumRange_m()); 
 
	this->supportedCompression.copy(source.getSupportedCompression()); 
 
	this->coordinateTransformationSupported.setName("CoordinateTransformationSupported");
	this->coordinateTransformationSupported.setOptional(true);
	this->coordinateTransformationSupported.setValue(source.getCoordinateTransformationSupported()); 
 
}

} // namespace environment
} // namespace openjaus

