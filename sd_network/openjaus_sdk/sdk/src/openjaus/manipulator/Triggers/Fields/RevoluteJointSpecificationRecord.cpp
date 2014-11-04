/**
\file RevoluteJointSpecificationRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/RevoluteJointSpecificationRecord.h"

namespace openjaus
{
namespace manipulator
{

RevoluteJointSpecificationRecord::RevoluteJointSpecificationRecord():
	linkLength_m(),
	twistAngle_rad(),
	jointOffset_m(),
	minValue_rad(),
	maxValue_rad(),
	maxSpeed_rps(),
	maxTorque()
{
	this->presenceVector = 0;

	fields.push_back(&linkLength_m);
	linkLength_m.setName("LinkLength");
	linkLength_m.setOptional(false);
	linkLength_m.setInterpretation("Link Length");
	// Nothing to init

	fields.push_back(&twistAngle_rad);
	twistAngle_rad.setName("TwistAngle");
	twistAngle_rad.setOptional(false);
	twistAngle_rad.setInterpretation("Twist Angle");
	// Nothing to init

	fields.push_back(&jointOffset_m);
	jointOffset_m.setName("JointOffset");
	jointOffset_m.setOptional(false);
	jointOffset_m.setInterpretation("Joint Offset");
	// Nothing to init

	fields.push_back(&minValue_rad);
	minValue_rad.setName("MinValue");
	minValue_rad.setOptional(true);
	minValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	// Nothing to init

	fields.push_back(&maxValue_rad);
	maxValue_rad.setName("MaxValue");
	maxValue_rad.setOptional(true);
	maxValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	// Nothing to init

	fields.push_back(&maxSpeed_rps);
	maxSpeed_rps.setName("MaxSpeed");
	maxSpeed_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&maxTorque);
	maxTorque.setName("MaxTorque");
	maxTorque.setOptional(true);
	// Nothing to init

}

RevoluteJointSpecificationRecord::RevoluteJointSpecificationRecord(const RevoluteJointSpecificationRecord &source)
{
	this->copy(const_cast<RevoluteJointSpecificationRecord&>(source));
}

RevoluteJointSpecificationRecord::~RevoluteJointSpecificationRecord()
{

}


double RevoluteJointSpecificationRecord::getLinkLength_m(void)
{
	return this->linkLength_m.getValue();
}

void RevoluteJointSpecificationRecord::setLinkLength_m(double value)
{
	this->linkLength_m.setValue(value);
}

double RevoluteJointSpecificationRecord::getTwistAngle_rad(void)
{
	return this->twistAngle_rad.getValue();
}

void RevoluteJointSpecificationRecord::setTwistAngle_rad(double value)
{
	this->twistAngle_rad.setValue(value);
}

double RevoluteJointSpecificationRecord::getJointOffset_m(void)
{
	return this->jointOffset_m.getValue();
}

void RevoluteJointSpecificationRecord::setJointOffset_m(double value)
{
	this->jointOffset_m.setValue(value);
}

double RevoluteJointSpecificationRecord::getMinValue_rad(void)
{
	return this->minValue_rad.getValue();
}

void RevoluteJointSpecificationRecord::setMinValue_rad(double value)
{
	this->minValue_rad.setValue(value);
}

double RevoluteJointSpecificationRecord::getMaxValue_rad(void)
{
	return this->maxValue_rad.getValue();
}

void RevoluteJointSpecificationRecord::setMaxValue_rad(double value)
{
	this->maxValue_rad.setValue(value);
}

double RevoluteJointSpecificationRecord::getMaxSpeed_rps(void)
{
	return this->maxSpeed_rps.getValue();
}

void RevoluteJointSpecificationRecord::setMaxSpeed_rps(double value)
{
	this->maxSpeed_rps.setValue(value);
}

double RevoluteJointSpecificationRecord::getMaxTorque(void)
{
	return this->maxTorque.getValue();
}

void RevoluteJointSpecificationRecord::setMaxTorque(double value)
{
	this->maxTorque.setValue(value);
}

int RevoluteJointSpecificationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(linkLength_m);
	byteSize += dst->pack(twistAngle_rad);
	byteSize += dst->pack(jointOffset_m);
	if(this->isMinValueEnabled())
	{
		byteSize += dst->pack(minValue_rad);
	}
	if(this->isMaxValueEnabled())
	{
		byteSize += dst->pack(maxValue_rad);
	}
	if(this->isMaxSpeedEnabled())
	{
		byteSize += dst->pack(maxSpeed_rps);
	}
	if(this->isMaxTorqueEnabled())
	{
		byteSize += dst->pack(maxTorque);
	}
	return byteSize;
}
int RevoluteJointSpecificationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(linkLength_m);
	byteSize += src->unpack(twistAngle_rad);
	byteSize += src->unpack(jointOffset_m);
	if(this->isMinValueEnabled())
	{
		byteSize += src->unpack(minValue_rad);
	}
	if(this->isMaxValueEnabled())
	{
		byteSize += src->unpack(maxValue_rad);
	}
	if(this->isMaxSpeedEnabled())
	{
		byteSize += src->unpack(maxSpeed_rps);
	}
	if(this->isMaxTorqueEnabled())
	{
		byteSize += src->unpack(maxTorque);
	}
	return byteSize;
}

int RevoluteJointSpecificationRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += linkLength_m.length(); // linkLength_m
	length += twistAngle_rad.length(); // twistAngle_rad
	length += jointOffset_m.length(); // jointOffset_m
	if(this->isMinValueEnabled())
	{
		length += minValue_rad.length(); // minValue_rad
	}
	if(this->isMaxValueEnabled())
	{
		length += maxValue_rad.length(); // maxValue_rad
	}
	if(this->isMaxSpeedEnabled())
	{
		length += maxSpeed_rps.length(); // maxSpeed_rps
	}
	if(this->isMaxTorqueEnabled())
	{
		length += maxTorque.length(); // maxTorque
	}
	return length;
}

std::string RevoluteJointSpecificationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RevoluteJointSpecificationRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMinValueEnabled value=\"" << std::boolalpha << this->isMinValueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaxValueEnabled value=\"" << std::boolalpha << this->isMaxValueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaxSpeedEnabled value=\"" << std::boolalpha << this->isMaxSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaxTorqueEnabled value=\"" << std::boolalpha << this->isMaxTorqueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << linkLength_m.toXml(level+1); // linkLength_m
	oss << twistAngle_rad.toXml(level+1); // twistAngle_rad
	oss << jointOffset_m.toXml(level+1); // jointOffset_m
	if(this->isMinValueEnabled())
	{
		oss << minValue_rad.toXml(level+1); // minValue_rad
	}
	if(this->isMaxValueEnabled())
	{
		oss << maxValue_rad.toXml(level+1); // maxValue_rad
	}
	if(this->isMaxSpeedEnabled())
	{
		oss << maxSpeed_rps.toXml(level+1); // maxSpeed_rps
	}
	if(this->isMaxTorqueEnabled())
	{
		oss << maxTorque.toXml(level+1); // maxTorque
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void RevoluteJointSpecificationRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t RevoluteJointSpecificationRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool RevoluteJointSpecificationRecord::isMinValueEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJointSpecificationRecord::MINVALUE_RAD));
}

void RevoluteJointSpecificationRecord::enableMinValue(void)
{
	this->presenceVector |= 0x01 << RevoluteJointSpecificationRecord::MINVALUE_RAD;
}

void RevoluteJointSpecificationRecord::disableMinValue(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJointSpecificationRecord::MINVALUE_RAD);
}

bool RevoluteJointSpecificationRecord::isMaxValueEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJointSpecificationRecord::MAXVALUE_RAD));
}

void RevoluteJointSpecificationRecord::enableMaxValue(void)
{
	this->presenceVector |= 0x01 << RevoluteJointSpecificationRecord::MAXVALUE_RAD;
}

void RevoluteJointSpecificationRecord::disableMaxValue(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJointSpecificationRecord::MAXVALUE_RAD);
}

bool RevoluteJointSpecificationRecord::isMaxSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJointSpecificationRecord::MAXSPEED_RPS));
}

void RevoluteJointSpecificationRecord::enableMaxSpeed(void)
{
	this->presenceVector |= 0x01 << RevoluteJointSpecificationRecord::MAXSPEED_RPS;
}

void RevoluteJointSpecificationRecord::disableMaxSpeed(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJointSpecificationRecord::MAXSPEED_RPS);
}

bool RevoluteJointSpecificationRecord::isMaxTorqueEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJointSpecificationRecord::MAXTORQUE));
}

void RevoluteJointSpecificationRecord::enableMaxTorque(void)
{
	this->presenceVector |= 0x01 << RevoluteJointSpecificationRecord::MAXTORQUE;
}

void RevoluteJointSpecificationRecord::disableMaxTorque(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJointSpecificationRecord::MAXTORQUE);
}


void RevoluteJointSpecificationRecord::copy(RevoluteJointSpecificationRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->linkLength_m.setName("LinkLength");
	this->linkLength_m.setOptional(false);
	this->linkLength_m.setInterpretation("Link Length");
	this->linkLength_m.setValue(source.getLinkLength_m()); 
 
	this->twistAngle_rad.setName("TwistAngle");
	this->twistAngle_rad.setOptional(false);
	this->twistAngle_rad.setInterpretation("Twist Angle");
	this->twistAngle_rad.setValue(source.getTwistAngle_rad()); 
 
	this->jointOffset_m.setName("JointOffset");
	this->jointOffset_m.setOptional(false);
	this->jointOffset_m.setInterpretation("Joint Offset");
	this->jointOffset_m.setValue(source.getJointOffset_m()); 
 
	this->minValue_rad.setName("MinValue");
	this->minValue_rad.setOptional(true);
	this->minValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	this->minValue_rad.setValue(source.getMinValue_rad()); 
 
	this->maxValue_rad.setName("MaxValue");
	this->maxValue_rad.setOptional(true);
	this->maxValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	this->maxValue_rad.setValue(source.getMaxValue_rad()); 
 
	this->maxSpeed_rps.setName("MaxSpeed");
	this->maxSpeed_rps.setOptional(true);
	this->maxSpeed_rps.setValue(source.getMaxSpeed_rps()); 
 
	this->maxTorque.setName("MaxTorque");
	this->maxTorque.setOptional(true);
	this->maxTorque.setValue(source.getMaxTorque()); 
 
}

} // namespace manipulator
} // namespace openjaus

