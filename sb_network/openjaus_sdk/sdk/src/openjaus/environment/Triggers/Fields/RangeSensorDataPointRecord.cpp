/**
\file RangeSensorDataPointRecord.h

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
#include "openjaus/environment/Triggers/Fields/RangeSensorDataPointRecord.h"

namespace openjaus
{
namespace environment
{

RangeSensorDataPointRecord::RangeSensorDataPointRecord():
	pointID(),
	range_m(),
	rangeValidity(),
	rangeErrorRMS_m(),
	bearing_rad(),
	bearingValidity(),
	bearingErrorRMS_rad(),
	inclination_rad(),
	inclinationValidity(),
	inclinationErrorRMS_rad()
{
	this->presenceVector = 0;

	fields.push_back(&pointID);
	pointID.setName("PointID");
	pointID.setOptional(true);
	pointID.setValue(0);

	fields.push_back(&range_m);
	range_m.setName("Range");
	range_m.setOptional(false);
	// Nothing to init

	fields.push_back(&rangeValidity);
	rangeValidity.setName("RangeValidity");
	rangeValidity.setOptional(true);
	// Nothing to init

	fields.push_back(&rangeErrorRMS_m);
	rangeErrorRMS_m.setName("RangeErrorRMS");
	rangeErrorRMS_m.setOptional(true);
	// Nothing to init

	fields.push_back(&bearing_rad);
	bearing_rad.setName("Bearing");
	bearing_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&bearingValidity);
	bearingValidity.setName("BearingValidity");
	bearingValidity.setOptional(true);
	// Nothing to init

	fields.push_back(&bearingErrorRMS_rad);
	bearingErrorRMS_rad.setName("BearingErrorRMS");
	bearingErrorRMS_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&inclination_rad);
	inclination_rad.setName("Inclination");
	inclination_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&inclinationValidity);
	inclinationValidity.setName("InclinationValidity");
	inclinationValidity.setOptional(true);
	// Nothing to init

	fields.push_back(&inclinationErrorRMS_rad);
	inclinationErrorRMS_rad.setName("InclinationErrorRMS");
	inclinationErrorRMS_rad.setOptional(true);
	// Nothing to init

}

RangeSensorDataPointRecord::RangeSensorDataPointRecord(const RangeSensorDataPointRecord &source)
{
	this->copy(const_cast<RangeSensorDataPointRecord&>(source));
}

RangeSensorDataPointRecord::~RangeSensorDataPointRecord()
{

}


uint32_t RangeSensorDataPointRecord::getPointID(void)
{
	return this->pointID.getValue();
}

void RangeSensorDataPointRecord::setPointID(uint32_t value)
{
	this->pointID.setValue(value);
}

double RangeSensorDataPointRecord::getRange_m(void)
{
	return this->range_m.getValue();
}

void RangeSensorDataPointRecord::setRange_m(double value)
{
	this->range_m.setValue(value);
}

RangeValidityEnumeration::RangeValidityEnum RangeSensorDataPointRecord::getRangeValidity(void)
{
	return this->rangeValidity.getValue();
}

void RangeSensorDataPointRecord::setRangeValidity(RangeValidityEnumeration::RangeValidityEnum value)
{
	this->rangeValidity.setValue(value);
}

double RangeSensorDataPointRecord::getRangeErrorRMS_m(void)
{
	return this->rangeErrorRMS_m.getValue();
}

void RangeSensorDataPointRecord::setRangeErrorRMS_m(double value)
{
	this->rangeErrorRMS_m.setValue(value);
}

double RangeSensorDataPointRecord::getBearing_rad(void)
{
	return this->bearing_rad.getValue();
}

void RangeSensorDataPointRecord::setBearing_rad(double value)
{
	this->bearing_rad.setValue(value);
}

BearingValidityEnumeration::BearingValidityEnum RangeSensorDataPointRecord::getBearingValidity(void)
{
	return this->bearingValidity.getValue();
}

void RangeSensorDataPointRecord::setBearingValidity(BearingValidityEnumeration::BearingValidityEnum value)
{
	this->bearingValidity.setValue(value);
}

double RangeSensorDataPointRecord::getBearingErrorRMS_rad(void)
{
	return this->bearingErrorRMS_rad.getValue();
}

void RangeSensorDataPointRecord::setBearingErrorRMS_rad(double value)
{
	this->bearingErrorRMS_rad.setValue(value);
}

double RangeSensorDataPointRecord::getInclination_rad(void)
{
	return this->inclination_rad.getValue();
}

void RangeSensorDataPointRecord::setInclination_rad(double value)
{
	this->inclination_rad.setValue(value);
}

InclinationValidityEnumeration::InclinationValidityEnum RangeSensorDataPointRecord::getInclinationValidity(void)
{
	return this->inclinationValidity.getValue();
}

void RangeSensorDataPointRecord::setInclinationValidity(InclinationValidityEnumeration::InclinationValidityEnum value)
{
	this->inclinationValidity.setValue(value);
}

double RangeSensorDataPointRecord::getInclinationErrorRMS_rad(void)
{
	return this->inclinationErrorRMS_rad.getValue();
}

void RangeSensorDataPointRecord::setInclinationErrorRMS_rad(double value)
{
	this->inclinationErrorRMS_rad.setValue(value);
}

int RangeSensorDataPointRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	if(this->isPointIDEnabled())
	{
		byteSize += dst->pack(pointID);
	}
	byteSize += dst->pack(range_m);
	if(this->isRangeValidityEnabled())
	{
		byteSize += dst->pack(rangeValidity);
	}
	if(this->isRangeErrorRMSEnabled())
	{
		byteSize += dst->pack(rangeErrorRMS_m);
	}
	byteSize += dst->pack(bearing_rad);
	if(this->isBearingValidityEnabled())
	{
		byteSize += dst->pack(bearingValidity);
	}
	if(this->isBearingErrorRMSEnabled())
	{
		byteSize += dst->pack(bearingErrorRMS_rad);
	}
	byteSize += dst->pack(inclination_rad);
	if(this->isInclinationValidityEnabled())
	{
		byteSize += dst->pack(inclinationValidity);
	}
	if(this->isInclinationErrorRMSEnabled())
	{
		byteSize += dst->pack(inclinationErrorRMS_rad);
	}
	return byteSize;
}
int RangeSensorDataPointRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	if(this->isPointIDEnabled())
	{
		byteSize += src->unpack(pointID);
	}
	byteSize += src->unpack(range_m);
	if(this->isRangeValidityEnabled())
	{
		byteSize += src->unpack(rangeValidity);
	}
	if(this->isRangeErrorRMSEnabled())
	{
		byteSize += src->unpack(rangeErrorRMS_m);
	}
	byteSize += src->unpack(bearing_rad);
	if(this->isBearingValidityEnabled())
	{
		byteSize += src->unpack(bearingValidity);
	}
	if(this->isBearingErrorRMSEnabled())
	{
		byteSize += src->unpack(bearingErrorRMS_rad);
	}
	byteSize += src->unpack(inclination_rad);
	if(this->isInclinationValidityEnabled())
	{
		byteSize += src->unpack(inclinationValidity);
	}
	if(this->isInclinationErrorRMSEnabled())
	{
		byteSize += src->unpack(inclinationErrorRMS_rad);
	}
	return byteSize;
}

int RangeSensorDataPointRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	if(this->isPointIDEnabled())
	{
		length += pointID.length(); // pointID
	}
	length += range_m.length(); // range_m
	if(this->isRangeValidityEnabled())
	{
		length += rangeValidity.length(); // rangeValidity
	}
	if(this->isRangeErrorRMSEnabled())
	{
		length += rangeErrorRMS_m.length(); // rangeErrorRMS_m
	}
	length += bearing_rad.length(); // bearing_rad
	if(this->isBearingValidityEnabled())
	{
		length += bearingValidity.length(); // bearingValidity
	}
	if(this->isBearingErrorRMSEnabled())
	{
		length += bearingErrorRMS_rad.length(); // bearingErrorRMS_rad
	}
	length += inclination_rad.length(); // inclination_rad
	if(this->isInclinationValidityEnabled())
	{
		length += inclinationValidity.length(); // inclinationValidity
	}
	if(this->isInclinationErrorRMSEnabled())
	{
		length += inclinationErrorRMS_rad.length(); // inclinationErrorRMS_rad
	}
	return length;
}

std::string RangeSensorDataPointRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RangeSensorDataPointRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPointIDEnabled value=\"" << std::boolalpha << this->isPointIDEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRangeValidityEnabled value=\"" << std::boolalpha << this->isRangeValidityEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRangeErrorRMSEnabled value=\"" << std::boolalpha << this->isRangeErrorRMSEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isBearingValidityEnabled value=\"" << std::boolalpha << this->isBearingValidityEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isBearingErrorRMSEnabled value=\"" << std::boolalpha << this->isBearingErrorRMSEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isInclinationValidityEnabled value=\"" << std::boolalpha << this->isInclinationValidityEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isInclinationErrorRMSEnabled value=\"" << std::boolalpha << this->isInclinationErrorRMSEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isPointIDEnabled())
	{
		oss << pointID.toXml(level+1); // pointID
	}
	oss << range_m.toXml(level+1); // range_m
	if(this->isRangeValidityEnabled())
	{
		oss << rangeValidity.toXml(level+1); // rangeValidity
	}
	if(this->isRangeErrorRMSEnabled())
	{
		oss << rangeErrorRMS_m.toXml(level+1); // rangeErrorRMS_m
	}
	oss << bearing_rad.toXml(level+1); // bearing_rad
	if(this->isBearingValidityEnabled())
	{
		oss << bearingValidity.toXml(level+1); // bearingValidity
	}
	if(this->isBearingErrorRMSEnabled())
	{
		oss << bearingErrorRMS_rad.toXml(level+1); // bearingErrorRMS_rad
	}
	oss << inclination_rad.toXml(level+1); // inclination_rad
	if(this->isInclinationValidityEnabled())
	{
		oss << inclinationValidity.toXml(level+1); // inclinationValidity
	}
	if(this->isInclinationErrorRMSEnabled())
	{
		oss << inclinationErrorRMS_rad.toXml(level+1); // inclinationErrorRMS_rad
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void RangeSensorDataPointRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t RangeSensorDataPointRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool RangeSensorDataPointRecord::isPointIDEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::POINTID));
}

void RangeSensorDataPointRecord::enablePointID(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::POINTID;
}

void RangeSensorDataPointRecord::disablePointID(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::POINTID);
}

bool RangeSensorDataPointRecord::isRangeValidityEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::RANGEVALIDITY));
}

void RangeSensorDataPointRecord::enableRangeValidity(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::RANGEVALIDITY;
}

void RangeSensorDataPointRecord::disableRangeValidity(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::RANGEVALIDITY);
}

bool RangeSensorDataPointRecord::isRangeErrorRMSEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::RANGEERRORRMS_M));
}

void RangeSensorDataPointRecord::enableRangeErrorRMS(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::RANGEERRORRMS_M;
}

void RangeSensorDataPointRecord::disableRangeErrorRMS(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::RANGEERRORRMS_M);
}

bool RangeSensorDataPointRecord::isBearingValidityEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::BEARINGVALIDITY));
}

void RangeSensorDataPointRecord::enableBearingValidity(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::BEARINGVALIDITY;
}

void RangeSensorDataPointRecord::disableBearingValidity(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::BEARINGVALIDITY);
}

bool RangeSensorDataPointRecord::isBearingErrorRMSEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::BEARINGERRORRMS_RAD));
}

void RangeSensorDataPointRecord::enableBearingErrorRMS(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::BEARINGERRORRMS_RAD;
}

void RangeSensorDataPointRecord::disableBearingErrorRMS(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::BEARINGERRORRMS_RAD);
}

bool RangeSensorDataPointRecord::isInclinationValidityEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::INCLINATIONVALIDITY));
}

void RangeSensorDataPointRecord::enableInclinationValidity(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::INCLINATIONVALIDITY;
}

void RangeSensorDataPointRecord::disableInclinationValidity(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::INCLINATIONVALIDITY);
}

bool RangeSensorDataPointRecord::isInclinationErrorRMSEnabled(void) const
{
	return (this->presenceVector & (0x01 << RangeSensorDataPointRecord::INCLINATIONERRORRMS_RAD));
}

void RangeSensorDataPointRecord::enableInclinationErrorRMS(void)
{
	this->presenceVector |= 0x01 << RangeSensorDataPointRecord::INCLINATIONERRORRMS_RAD;
}

void RangeSensorDataPointRecord::disableInclinationErrorRMS(void)
{
	this->presenceVector &= ~(0x01 << RangeSensorDataPointRecord::INCLINATIONERRORRMS_RAD);
}


void RangeSensorDataPointRecord::copy(RangeSensorDataPointRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->pointID.setName("PointID");
	this->pointID.setOptional(true);
	this->pointID.setValue(source.getPointID()); 
 
	this->range_m.setName("Range");
	this->range_m.setOptional(false);
	this->range_m.setValue(source.getRange_m()); 
 
	this->rangeValidity.setName("RangeValidity");
	this->rangeValidity.setOptional(true);
	this->rangeValidity.setValue(source.getRangeValidity()); 
 
	this->rangeErrorRMS_m.setName("RangeErrorRMS");
	this->rangeErrorRMS_m.setOptional(true);
	this->rangeErrorRMS_m.setValue(source.getRangeErrorRMS_m()); 
 
	this->bearing_rad.setName("Bearing");
	this->bearing_rad.setOptional(false);
	this->bearing_rad.setValue(source.getBearing_rad()); 
 
	this->bearingValidity.setName("BearingValidity");
	this->bearingValidity.setOptional(true);
	this->bearingValidity.setValue(source.getBearingValidity()); 
 
	this->bearingErrorRMS_rad.setName("BearingErrorRMS");
	this->bearingErrorRMS_rad.setOptional(true);
	this->bearingErrorRMS_rad.setValue(source.getBearingErrorRMS_rad()); 
 
	this->inclination_rad.setName("Inclination");
	this->inclination_rad.setOptional(false);
	this->inclination_rad.setValue(source.getInclination_rad()); 
 
	this->inclinationValidity.setName("InclinationValidity");
	this->inclinationValidity.setOptional(true);
	this->inclinationValidity.setValue(source.getInclinationValidity()); 
 
	this->inclinationErrorRMS_rad.setName("InclinationErrorRMS");
	this->inclinationErrorRMS_rad.setOptional(true);
	this->inclinationErrorRMS_rad.setValue(source.getInclinationErrorRMS_rad()); 
 
}

} // namespace environment
} // namespace openjaus

