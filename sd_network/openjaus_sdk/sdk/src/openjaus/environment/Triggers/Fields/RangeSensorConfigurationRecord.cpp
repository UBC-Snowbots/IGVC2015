/**
\file RangeSensorConfigurationRecord.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorConfigurationRecord.h"

namespace openjaus
{
namespace environment
{

RangeSensorConfigurationRecord::RangeSensorConfigurationRecord():
	sensorID(),
	horizontalFieldOfViewStartAngle_rad(),
	horizontalFieldOfViewStopAngle_rad(),
	verticalFieldOfViewStartAngle_rad(),
	verticalFieldOfViewStopAngle_rad(),
	updateRate_Hz(),
	minimumRange_m(),
	maximumRange_m(),
	sensorState()
{
	this->presenceVector = 0;

	fields.push_back(&sensorID);
	sensorID.setName("SensorID");
	sensorID.setOptional(false);
	sensorID.setValue(0);

	fields.push_back(&horizontalFieldOfViewStartAngle_rad);
	horizontalFieldOfViewStartAngle_rad.setName("HorizontalFieldOfViewStartAngle");
	horizontalFieldOfViewStartAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&horizontalFieldOfViewStopAngle_rad);
	horizontalFieldOfViewStopAngle_rad.setName("HorizontalFieldOfViewStopAngle");
	horizontalFieldOfViewStopAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&verticalFieldOfViewStartAngle_rad);
	verticalFieldOfViewStartAngle_rad.setName("VerticalFieldOfViewStartAngle");
	verticalFieldOfViewStartAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&verticalFieldOfViewStopAngle_rad);
	verticalFieldOfViewStopAngle_rad.setName("VerticalFieldOfViewStopAngle");
	verticalFieldOfViewStopAngle_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&updateRate_Hz);
	updateRate_Hz.setName("UpdateRate");
	updateRate_Hz.setOptional(true);
	// Nothing to init

	fields.push_back(&minimumRange_m);
	minimumRange_m.setName("MinimumRange");
	minimumRange_m.setOptional(true);
	// Nothing to init

	fields.push_back(&maximumRange_m);
	maximumRange_m.setName("MaximumRange");
	maximumRange_m.setOptional(true);
	// Nothing to init

	fields.push_back(&sensorState);
	sensorState.setName("SensorState");
	sensorState.setOptional(true);
	// Nothing to init

}

RangeSensorConfigurationRecord::RangeSensorConfigurationRecord(const RangeSensorConfigurationRecord &source)
{
	this->copy(const_cast<RangeSensorConfigurationRecord&>(source));
}

RangeSensorConfigurationRecord::~RangeSensorConfigurationRecord()
{

}


uint16_t RangeSensorConfigurationRecord::getSensorID(void)
{
	return this->sensorID.getValue();
}

void RangeSensorConfigurationRecord::setSensorID(uint16_t value)
{
	this->sensorID.setValue(value);
}

double RangeSensorConfigurationRecord::getHorizontalFieldOfViewStartAngle_rad(void)
{
	return this->horizontalFieldOfViewStartAngle_rad.getValue();
}

void RangeSensorConfigurationRecord::setHorizontalFieldOfViewStartAngle_rad(double value)
{
	this->horizontalFieldOfViewStartAngle_rad.setValue(value);
}

double RangeSensorConfigurationRecord::getHorizontalFieldOfViewStopAngle_rad(void)
{
	return this->horizontalFieldOfViewStopAngle_rad.getValue();
}

void RangeSensorConfigurationRecord::setHorizontalFieldOfViewStopAngle_rad(double value)
{
	this->horizontalFieldOfViewStopAngle_rad.setValue(value);
}

double RangeSensorConfigurationRecord::getVerticalFieldOfViewStartAngle_rad(void)
{
	return this->verticalFieldOfViewStartAngle_rad.getValue();
}

void RangeSensorConfigurationRecord::setVerticalFieldOfViewStartAngle_rad(double value)
{
	this->verticalFieldOfViewStartAngle_rad.setValue(value);
}

double RangeSensorConfigurationRecord::getVerticalFieldOfViewStopAngle_rad(void)
{
	return this->verticalFieldOfViewStopAngle_rad.getValue();
}

void RangeSensorConfigurationRecord::setVerticalFieldOfViewStopAngle_rad(double value)
{
	this->verticalFieldOfViewStopAngle_rad.setValue(value);
}

double RangeSensorConfigurationRecord::getUpdateRate_Hz(void)
{
	return this->updateRate_Hz.getValue();
}

void RangeSensorConfigurationRecord::setUpdateRate_Hz(double value)
{
	this->updateRate_Hz.setValue(value);
}

double RangeSensorConfigurationRecord::getMinimumRange_m(void)
{
	return this->minimumRange_m.getValue();
}

void RangeSensorConfigurationRecord::setMinimumRange_m(double value)
{
	this->minimumRange_m.setValue(value);
}

double RangeSensorConfigurationRecord::getMaximumRange_m(void)
{
	return this->maximumRange_m.getValue();
}

void RangeSensorConfigurationRecord::setMaximumRange_m(double value)
{
	this->maximumRange_m.setValue(value);
}

SensorStateEnumeration::SensorStateEnum RangeSensorConfigurationRecord::getSensorState(void)
{
	return this->sensorState.getValue();
}

void RangeSensorConfigurationRecord::setSensorState(SensorStateEnumeration::SensorStateEnum value)
{
	this->sensorState.setValue(value);
}

int RangeSensorConfigurationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(sensorID);
	if(this->isHorizontalFieldOfViewStartAngleEnabled())
	{
		byteSize += dst->pack(horizontalFieldOfViewStartAngle_rad);
	}
	if(this->isHorizontalFieldOfViewStopAngleEnabled())
	{
		byteSize += dst->pack(horizontalFieldOfViewStopAngle_rad);
	}
	if(this->isVerticalFieldOfViewStartAngleEnabled())
	{
		byteSize += dst->pack(verticalFieldOfViewStartAngle_rad);
	}
	if(this->isVerticalFieldOfViewStopAngleEnabled())
	{
		byteSize += dst->pack(verticalFieldOfViewStopAngle_rad);
	}
	if(this->isUpdateRateEnabled())
	{
		byteSize += dst->pack(updateRate_Hz);
	}
	if(this->isMinimumRangeEnabled())
	{
		byteSize += dst->pack(minimumRange_m);
	}
	if(this->isMaximumRangeEnabled())
	{
		byteSize += dst->pack(maximumRange_m);
	}
	if(this->isSensorStateEnabled())
	{
		byteSize += dst->pack(sensorState);
	}
	return byteSize;
}
int RangeSensorConfigurationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(sensorID);
	if(this->isHorizontalFieldOfViewStartAngleEnabled())
	{
		byteSize += src->unpack(horizontalFieldOfViewStartAngle_rad);
	}
	if(this->isHorizontalFieldOfViewStopAngleEnabled())
	{
		byteSize += src->unpack(horizontalFieldOfViewStopAngle_rad);
	}
	if(this->isVerticalFieldOfViewStartAngleEnabled())
	{
		byteSize += src->unpack(verticalFieldOfViewStartAngle_rad);
	}
	if(this->isVerticalFieldOfViewStopAngleEnabled())
	{
		byteSize += src->unpack(verticalFieldOfViewStopAngle_rad);
	}
	if(this->isUpdateRateEnabled())
	{
		byteSize += src->unpack(updateRate_Hz);
	}
	if(this->isMinimumRangeEnabled())
	{
		byteSize += src->unpack(minimumRange_m);
	}
	if(this->isMaximumRangeEnabled())
	{
		byteSize += src->unpack(maximumRange_m);
	}
	if(this->isSensorStateEnabled())
	{
		byteSize += src->unpack(sensorState);
	}
	return byteSize;
}

int RangeSensorConfigurationRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += sensorID.length(); // sensorID
	if(this->isHorizontalFieldOfViewStartAngleEnabled())
	{
		length += horizontalFieldOfViewStartAngle_rad.length(); // horizontalFieldOfViewStartAngle_rad
	}
	if(this->isHorizontalFieldOfViewStopAngleEnabled())
	{
		length += horizontalFieldOfViewStopAngle_rad.length(); // horizontalFieldOfViewStopAngle_rad
	}
	if(this->isVerticalFieldOfViewStartAngleEnabled())
	{
		length += verticalFieldOfViewStartAngle_rad.length(); // verticalFieldOfViewStartAngle_rad
	}
	if(this->isVerticalFieldOfViewStopAngleEnabled())
	{
		length += verticalFieldOfViewStopAngle_rad.length(); // verticalFieldOfViewStopAngle_rad
	}
	if(this->isUpdateRateEnabled())
	{
		length += updateRate_Hz.length(); // updateRate_Hz
	}
	if(this->isMinimumRangeEnabled())
	{
		length += minimumRange_m.length(); // minimumRange_m
	}
	if(this->isMaximumRangeEnabled())
	{
		length += maximumRange_m.length(); // maximumRange_m
	}
	if(this->isSensorStateEnabled())
	{
		length += sensorState.length(); // sensorState
	}
	return length;
}

std::string RangeSensorConfigurationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RangeSensorConfigurationRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isHorizontalFieldOfViewStartAngleEnabled value=\"" << std::boolalpha << this->isHorizontalFieldOfViewStartAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isHorizontalFieldOfViewStopAngleEnabled value=\"" << std::boolalpha << this->isHorizontalFieldOfViewStopAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVerticalFieldOfViewStartAngleEnabled value=\"" << std::boolalpha << this->isVerticalFieldOfViewStartAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVerticalFieldOfViewStopAngleEnabled value=\"" << std::boolalpha << this->isVerticalFieldOfViewStopAngleEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isUpdateRateEnabled value=\"" << std::boolalpha << this->isUpdateRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinimumRangeEnabled value=\"" << std::boolalpha << this->isMinimumRangeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaximumRangeEnabled value=\"" << std::boolalpha << this->isMaximumRangeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSensorStateEnabled value=\"" << std::boolalpha << this->isSensorStateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << sensorID.toXml(level+1); // sensorID
	if(this->isHorizontalFieldOfViewStartAngleEnabled())
	{
		oss << horizontalFieldOfViewStartAngle_rad.toXml(level+1); // horizontalFieldOfViewStartAngle_rad
	}
	if(this->isHorizontalFieldOfViewStopAngleEnabled())
	{
		oss << horizontalFieldOfViewStopAngle_rad.toXml(level+1); // horizontalFieldOfViewStopAngle_rad
	}
	if(this->isVerticalFieldOfViewStartAngleEnabled())
	{
		oss << verticalFieldOfViewStartAngle_rad.toXml(level+1); // verticalFieldOfViewStartAngle_rad
	}
	if(this->isVerticalFieldOfViewStopAngleEnabled())
	{
		oss << verticalFieldOfViewStopAngle_rad.toXml(level+1); // verticalFieldOfViewStopAngle_rad
	}
	if(this->isUpdateRateEnabled())
	{
		oss << updateRate_Hz.toXml(level+1); // updateRate_Hz
	}
	if(this->isMinimumRangeEnabled())
	{
		oss << minimumRange_m.toXml(level+1); // minimumRange_m
	}
	if(this->isMaximumRangeEnabled())
	{
		oss << maximumRange_m.toXml(level+1); // maximumRange_m
	}
	if(this->isSensorStateEnabled())
	{
		oss << sensorState.toXml(level+1); // sensorState
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void RangeSensorConfigurationRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t RangeSensorConfigurationRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool RangeSensorConfigurationRecord::isHorizontalFieldOfViewStartAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::HORIZONTALFIELDOFVIEWSTARTANGLE_RAD));
}

void RangeSensorConfigurationRecord::enableHorizontalFieldOfViewStartAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::HORIZONTALFIELDOFVIEWSTARTANGLE_RAD;
}

void RangeSensorConfigurationRecord::disableHorizontalFieldOfViewStartAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::HORIZONTALFIELDOFVIEWSTARTANGLE_RAD);
}

bool RangeSensorConfigurationRecord::isHorizontalFieldOfViewStopAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::HORIZONTALFIELDOFVIEWSTOPANGLE_RAD));
}

void RangeSensorConfigurationRecord::enableHorizontalFieldOfViewStopAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::HORIZONTALFIELDOFVIEWSTOPANGLE_RAD;
}

void RangeSensorConfigurationRecord::disableHorizontalFieldOfViewStopAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::HORIZONTALFIELDOFVIEWSTOPANGLE_RAD);
}

bool RangeSensorConfigurationRecord::isVerticalFieldOfViewStartAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::VERTICALFIELDOFVIEWSTARTANGLE_RAD));
}

void RangeSensorConfigurationRecord::enableVerticalFieldOfViewStartAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::VERTICALFIELDOFVIEWSTARTANGLE_RAD;
}

void RangeSensorConfigurationRecord::disableVerticalFieldOfViewStartAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::VERTICALFIELDOFVIEWSTARTANGLE_RAD);
}

bool RangeSensorConfigurationRecord::isVerticalFieldOfViewStopAngleEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::VERTICALFIELDOFVIEWSTOPANGLE_RAD));
}

void RangeSensorConfigurationRecord::enableVerticalFieldOfViewStopAngle(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::VERTICALFIELDOFVIEWSTOPANGLE_RAD;
}

void RangeSensorConfigurationRecord::disableVerticalFieldOfViewStopAngle(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::VERTICALFIELDOFVIEWSTOPANGLE_RAD);
}

bool RangeSensorConfigurationRecord::isUpdateRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::UPDATERATE_HZ));
}

void RangeSensorConfigurationRecord::enableUpdateRate(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::UPDATERATE_HZ;
}

void RangeSensorConfigurationRecord::disableUpdateRate(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::UPDATERATE_HZ);
}

bool RangeSensorConfigurationRecord::isMinimumRangeEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::MINIMUMRANGE_M));
}

void RangeSensorConfigurationRecord::enableMinimumRange(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::MINIMUMRANGE_M;
}

void RangeSensorConfigurationRecord::disableMinimumRange(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::MINIMUMRANGE_M);
}

bool RangeSensorConfigurationRecord::isMaximumRangeEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::MAXIMUMRANGE_M));
}

void RangeSensorConfigurationRecord::enableMaximumRange(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::MAXIMUMRANGE_M;
}

void RangeSensorConfigurationRecord::disableMaximumRange(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::MAXIMUMRANGE_M);
}

bool RangeSensorConfigurationRecord::isSensorStateEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorConfigurationRecord::SENSORSTATE));
}

void RangeSensorConfigurationRecord::enableSensorState(void)
{
	this->presenceVector |= 0x01 << RangeSensorConfigurationRecord::SENSORSTATE;
}

void RangeSensorConfigurationRecord::disableSensorState(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorConfigurationRecord::SENSORSTATE);
}


void RangeSensorConfigurationRecord::copy(RangeSensorConfigurationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->sensorID.setName("SensorID");
	this->sensorID.setOptional(false);
	this->sensorID.setValue(source.getSensorID()); 
 
	this->horizontalFieldOfViewStartAngle_rad.setName("HorizontalFieldOfViewStartAngle");
	this->horizontalFieldOfViewStartAngle_rad.setOptional(true);
	this->horizontalFieldOfViewStartAngle_rad.setValue(source.getHorizontalFieldOfViewStartAngle_rad()); 
 
	this->horizontalFieldOfViewStopAngle_rad.setName("HorizontalFieldOfViewStopAngle");
	this->horizontalFieldOfViewStopAngle_rad.setOptional(true);
	this->horizontalFieldOfViewStopAngle_rad.setValue(source.getHorizontalFieldOfViewStopAngle_rad()); 
 
	this->verticalFieldOfViewStartAngle_rad.setName("VerticalFieldOfViewStartAngle");
	this->verticalFieldOfViewStartAngle_rad.setOptional(true);
	this->verticalFieldOfViewStartAngle_rad.setValue(source.getVerticalFieldOfViewStartAngle_rad()); 
 
	this->verticalFieldOfViewStopAngle_rad.setName("VerticalFieldOfViewStopAngle");
	this->verticalFieldOfViewStopAngle_rad.setOptional(true);
	this->verticalFieldOfViewStopAngle_rad.setValue(source.getVerticalFieldOfViewStopAngle_rad()); 
 
	this->updateRate_Hz.setName("UpdateRate");
	this->updateRate_Hz.setOptional(true);
	this->updateRate_Hz.setValue(source.getUpdateRate_Hz()); 
 
	this->minimumRange_m.setName("MinimumRange");
	this->minimumRange_m.setOptional(true);
	this->minimumRange_m.setValue(source.getMinimumRange_m()); 
 
	this->maximumRange_m.setName("MaximumRange");
	this->maximumRange_m.setOptional(true);
	this->maximumRange_m.setValue(source.getMaximumRange_m()); 
 
	this->sensorState.setName("SensorState");
	this->sensorState.setOptional(true);
	this->sensorState.setValue(source.getSensorState()); 
 
}

} // namespace environment
} // namespace openjaus

