/**
\file RevoluteJoint1OffsetRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/RevoluteJoint1OffsetRecord.h"

namespace openjaus
{
namespace manipulator
{

RevoluteJoint1OffsetRecord::RevoluteJoint1OffsetRecord():
	joint1Offset_m(),
	joint1MinValue_rad(),
	joint1MaxValue_rad(),
	joint1MaxSpeed_rps(),
	joint1MaxTorque()
{
	this->presenceVector = 0;

	fields.push_back(&joint1Offset_m);
	joint1Offset_m.setName("Joint1Offset");
	joint1Offset_m.setOptional(false);
	joint1Offset_m.setInterpretation("Joint Offset");
	// Nothing to init

	fields.push_back(&joint1MinValue_rad);
	joint1MinValue_rad.setName("Joint1MinValue");
	joint1MinValue_rad.setOptional(true);
	joint1MinValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	// Nothing to init

	fields.push_back(&joint1MaxValue_rad);
	joint1MaxValue_rad.setName("Joint1MaxValue");
	joint1MaxValue_rad.setOptional(true);
	joint1MaxValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	// Nothing to init

	fields.push_back(&joint1MaxSpeed_rps);
	joint1MaxSpeed_rps.setName("Joint1MaxSpeed");
	joint1MaxSpeed_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&joint1MaxTorque);
	joint1MaxTorque.setName("Joint1MaxTorque");
	joint1MaxTorque.setOptional(true);
	// Nothing to init

}

RevoluteJoint1OffsetRecord::RevoluteJoint1OffsetRecord(const RevoluteJoint1OffsetRecord &source)
{
	this->copy(const_cast<RevoluteJoint1OffsetRecord&>(source));
}

RevoluteJoint1OffsetRecord::~RevoluteJoint1OffsetRecord()
{

}


double RevoluteJoint1OffsetRecord::getJoint1Offset_m(void)
{
	return this->joint1Offset_m.getValue();
}

void RevoluteJoint1OffsetRecord::setJoint1Offset_m(double value)
{
	this->joint1Offset_m.setValue(value);
}

double RevoluteJoint1OffsetRecord::getJoint1MinValue_rad(void)
{
	return this->joint1MinValue_rad.getValue();
}

void RevoluteJoint1OffsetRecord::setJoint1MinValue_rad(double value)
{
	this->joint1MinValue_rad.setValue(value);
}

double RevoluteJoint1OffsetRecord::getJoint1MaxValue_rad(void)
{
	return this->joint1MaxValue_rad.getValue();
}

void RevoluteJoint1OffsetRecord::setJoint1MaxValue_rad(double value)
{
	this->joint1MaxValue_rad.setValue(value);
}

double RevoluteJoint1OffsetRecord::getJoint1MaxSpeed_rps(void)
{
	return this->joint1MaxSpeed_rps.getValue();
}

void RevoluteJoint1OffsetRecord::setJoint1MaxSpeed_rps(double value)
{
	this->joint1MaxSpeed_rps.setValue(value);
}

double RevoluteJoint1OffsetRecord::getJoint1MaxTorque(void)
{
	return this->joint1MaxTorque.getValue();
}

void RevoluteJoint1OffsetRecord::setJoint1MaxTorque(double value)
{
	this->joint1MaxTorque.setValue(value);
}

int RevoluteJoint1OffsetRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(joint1Offset_m);
	if(this->isJoint1MinValueEnabled())
	{
		byteSize += dst->pack(joint1MinValue_rad);
	}
	if(this->isJoint1MaxValueEnabled())
	{
		byteSize += dst->pack(joint1MaxValue_rad);
	}
	if(this->isJoint1MaxSpeedEnabled())
	{
		byteSize += dst->pack(joint1MaxSpeed_rps);
	}
	if(this->isJoint1MaxTorqueEnabled())
	{
		byteSize += dst->pack(joint1MaxTorque);
	}
	return byteSize;
}
int RevoluteJoint1OffsetRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(joint1Offset_m);
	if(this->isJoint1MinValueEnabled())
	{
		byteSize += src->unpack(joint1MinValue_rad);
	}
	if(this->isJoint1MaxValueEnabled())
	{
		byteSize += src->unpack(joint1MaxValue_rad);
	}
	if(this->isJoint1MaxSpeedEnabled())
	{
		byteSize += src->unpack(joint1MaxSpeed_rps);
	}
	if(this->isJoint1MaxTorqueEnabled())
	{
		byteSize += src->unpack(joint1MaxTorque);
	}
	return byteSize;
}

int RevoluteJoint1OffsetRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += joint1Offset_m.length(); // joint1Offset_m
	if(this->isJoint1MinValueEnabled())
	{
		length += joint1MinValue_rad.length(); // joint1MinValue_rad
	}
	if(this->isJoint1MaxValueEnabled())
	{
		length += joint1MaxValue_rad.length(); // joint1MaxValue_rad
	}
	if(this->isJoint1MaxSpeedEnabled())
	{
		length += joint1MaxSpeed_rps.length(); // joint1MaxSpeed_rps
	}
	if(this->isJoint1MaxTorqueEnabled())
	{
		length += joint1MaxTorque.length(); // joint1MaxTorque
	}
	return length;
}

std::string RevoluteJoint1OffsetRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"RevoluteJoint1OffsetRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isJoint1MinValueEnabled value=\"" << std::boolalpha << this->isJoint1MinValueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isJoint1MaxValueEnabled value=\"" << std::boolalpha << this->isJoint1MaxValueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isJoint1MaxSpeedEnabled value=\"" << std::boolalpha << this->isJoint1MaxSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isJoint1MaxTorqueEnabled value=\"" << std::boolalpha << this->isJoint1MaxTorqueEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << joint1Offset_m.toXml(level+1); // joint1Offset_m
	if(this->isJoint1MinValueEnabled())
	{
		oss << joint1MinValue_rad.toXml(level+1); // joint1MinValue_rad
	}
	if(this->isJoint1MaxValueEnabled())
	{
		oss << joint1MaxValue_rad.toXml(level+1); // joint1MaxValue_rad
	}
	if(this->isJoint1MaxSpeedEnabled())
	{
		oss << joint1MaxSpeed_rps.toXml(level+1); // joint1MaxSpeed_rps
	}
	if(this->isJoint1MaxTorqueEnabled())
	{
		oss << joint1MaxTorque.toXml(level+1); // joint1MaxTorque
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void RevoluteJoint1OffsetRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t RevoluteJoint1OffsetRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool RevoluteJoint1OffsetRecord::isJoint1MinValueEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJoint1OffsetRecord::JOINT1MINVALUE_RAD));
}

void RevoluteJoint1OffsetRecord::enableJoint1MinValue(void)
{
	this->presenceVector |= 0x01 << RevoluteJoint1OffsetRecord::JOINT1MINVALUE_RAD;
}

void RevoluteJoint1OffsetRecord::disableJoint1MinValue(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJoint1OffsetRecord::JOINT1MINVALUE_RAD);
}

bool RevoluteJoint1OffsetRecord::isJoint1MaxValueEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXVALUE_RAD));
}

void RevoluteJoint1OffsetRecord::enableJoint1MaxValue(void)
{
	this->presenceVector |= 0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXVALUE_RAD;
}

void RevoluteJoint1OffsetRecord::disableJoint1MaxValue(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXVALUE_RAD);
}

bool RevoluteJoint1OffsetRecord::isJoint1MaxSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXSPEED_RPS));
}

void RevoluteJoint1OffsetRecord::enableJoint1MaxSpeed(void)
{
	this->presenceVector |= 0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXSPEED_RPS;
}

void RevoluteJoint1OffsetRecord::disableJoint1MaxSpeed(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXSPEED_RPS);
}

bool RevoluteJoint1OffsetRecord::isJoint1MaxTorqueEnabled(void) const
{
	return (this->presenceVector & (0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXTORQUE));
}

void RevoluteJoint1OffsetRecord::enableJoint1MaxTorque(void)
{
	this->presenceVector |= 0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXTORQUE;
}

void RevoluteJoint1OffsetRecord::disableJoint1MaxTorque(void)
{
	this->presenceVector &= ~(0x01 << RevoluteJoint1OffsetRecord::JOINT1MAXTORQUE);
}


void RevoluteJoint1OffsetRecord::copy(RevoluteJoint1OffsetRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->joint1Offset_m.setName("Joint1Offset");
	this->joint1Offset_m.setOptional(false);
	this->joint1Offset_m.setInterpretation("Joint Offset");
	this->joint1Offset_m.setValue(source.getJoint1Offset_m()); 
 
	this->joint1MinValue_rad.setName("Joint1MinValue");
	this->joint1MinValue_rad.setOptional(true);
	this->joint1MinValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	this->joint1MinValue_rad.setValue(source.getJoint1MinValue_rad()); 
 
	this->joint1MaxValue_rad.setName("Joint1MaxValue");
	this->joint1MaxValue_rad.setOptional(true);
	this->joint1MaxValue_rad.setInterpretation("Note: This field is omitted by using the presence vector for joints that can rotate continuously without limit.");
	this->joint1MaxValue_rad.setValue(source.getJoint1MaxValue_rad()); 
 
	this->joint1MaxSpeed_rps.setName("Joint1MaxSpeed");
	this->joint1MaxSpeed_rps.setOptional(true);
	this->joint1MaxSpeed_rps.setValue(source.getJoint1MaxSpeed_rps()); 
 
	this->joint1MaxTorque.setName("Joint1MaxTorque");
	this->joint1MaxTorque.setOptional(true);
	this->joint1MaxTorque.setValue(source.getJoint1MaxTorque()); 
 
}

} // namespace manipulator
} // namespace openjaus

