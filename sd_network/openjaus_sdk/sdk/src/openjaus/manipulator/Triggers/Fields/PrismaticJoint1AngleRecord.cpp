/**
\file PrismaticJoint1AngleRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/PrismaticJoint1AngleRecord.h"

namespace openjaus
{
namespace manipulator
{

PrismaticJoint1AngleRecord::PrismaticJoint1AngleRecord():
	joint1Angle_rad(),
	joint1MinValue_m(),
	joint1MaxValue_m(),
	joint1MaxSpeed_mps(),
	joint1MaxForce()
{
	this->presenceVector = 0;

	fields.push_back(&joint1Angle_rad);
	joint1Angle_rad.setName("Joint1Angle");
	joint1Angle_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MinValue_m);
	joint1MinValue_m.setName("Joint1MinValue");
	joint1MinValue_m.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxValue_m);
	joint1MaxValue_m.setName("Joint1MaxValue");
	joint1MaxValue_m.setOptional(false);
	// Nothing to init

	fields.push_back(&joint1MaxSpeed_mps);
	joint1MaxSpeed_mps.setName("Joint1MaxSpeed");
	joint1MaxSpeed_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&joint1MaxForce);
	joint1MaxForce.setName("Joint1MaxForce");
	joint1MaxForce.setOptional(true);
	// Nothing to init

}

PrismaticJoint1AngleRecord::PrismaticJoint1AngleRecord(const PrismaticJoint1AngleRecord &source)
{
	this->copy(const_cast<PrismaticJoint1AngleRecord&>(source));
}

PrismaticJoint1AngleRecord::~PrismaticJoint1AngleRecord()
{

}


double PrismaticJoint1AngleRecord::getJoint1Angle_rad(void)
{
	return this->joint1Angle_rad.getValue();
}

void PrismaticJoint1AngleRecord::setJoint1Angle_rad(double value)
{
	this->joint1Angle_rad.setValue(value);
}

double PrismaticJoint1AngleRecord::getJoint1MinValue_m(void)
{
	return this->joint1MinValue_m.getValue();
}

void PrismaticJoint1AngleRecord::setJoint1MinValue_m(double value)
{
	this->joint1MinValue_m.setValue(value);
}

double PrismaticJoint1AngleRecord::getJoint1MaxValue_m(void)
{
	return this->joint1MaxValue_m.getValue();
}

void PrismaticJoint1AngleRecord::setJoint1MaxValue_m(double value)
{
	this->joint1MaxValue_m.setValue(value);
}

double PrismaticJoint1AngleRecord::getJoint1MaxSpeed_mps(void)
{
	return this->joint1MaxSpeed_mps.getValue();
}

void PrismaticJoint1AngleRecord::setJoint1MaxSpeed_mps(double value)
{
	this->joint1MaxSpeed_mps.setValue(value);
}

double PrismaticJoint1AngleRecord::getJoint1MaxForce(void)
{
	return this->joint1MaxForce.getValue();
}

void PrismaticJoint1AngleRecord::setJoint1MaxForce(double value)
{
	this->joint1MaxForce.setValue(value);
}

int PrismaticJoint1AngleRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(joint1Angle_rad);
	byteSize += dst->pack(joint1MinValue_m);
	byteSize += dst->pack(joint1MaxValue_m);
	if(this->isJoint1MaxSpeedEnabled())
	{
		byteSize += dst->pack(joint1MaxSpeed_mps);
	}
	if(this->isJoint1MaxForceEnabled())
	{
		byteSize += dst->pack(joint1MaxForce);
	}
	return byteSize;
}
int PrismaticJoint1AngleRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(joint1Angle_rad);
	byteSize += src->unpack(joint1MinValue_m);
	byteSize += src->unpack(joint1MaxValue_m);
	if(this->isJoint1MaxSpeedEnabled())
	{
		byteSize += src->unpack(joint1MaxSpeed_mps);
	}
	if(this->isJoint1MaxForceEnabled())
	{
		byteSize += src->unpack(joint1MaxForce);
	}
	return byteSize;
}

int PrismaticJoint1AngleRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += joint1Angle_rad.length(); // joint1Angle_rad
	length += joint1MinValue_m.length(); // joint1MinValue_m
	length += joint1MaxValue_m.length(); // joint1MaxValue_m
	if(this->isJoint1MaxSpeedEnabled())
	{
		length += joint1MaxSpeed_mps.length(); // joint1MaxSpeed_mps
	}
	if(this->isJoint1MaxForceEnabled())
	{
		length += joint1MaxForce.length(); // joint1MaxForce
	}
	return length;
}

std::string PrismaticJoint1AngleRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"PrismaticJoint1AngleRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isJoint1MaxSpeedEnabled value=\"" << std::boolalpha << this->isJoint1MaxSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isJoint1MaxForceEnabled value=\"" << std::boolalpha << this->isJoint1MaxForceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << joint1Angle_rad.toXml(level+1); // joint1Angle_rad
	oss << joint1MinValue_m.toXml(level+1); // joint1MinValue_m
	oss << joint1MaxValue_m.toXml(level+1); // joint1MaxValue_m
	if(this->isJoint1MaxSpeedEnabled())
	{
		oss << joint1MaxSpeed_mps.toXml(level+1); // joint1MaxSpeed_mps
	}
	if(this->isJoint1MaxForceEnabled())
	{
		oss << joint1MaxForce.toXml(level+1); // joint1MaxForce
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void PrismaticJoint1AngleRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t PrismaticJoint1AngleRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool PrismaticJoint1AngleRecord::isJoint1MaxSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << PrismaticJoint1AngleRecord::JOINT1MAXSPEED_MPS));
}

void PrismaticJoint1AngleRecord::enableJoint1MaxSpeed(void)
{
	this->presenceVector |= 0x01 << PrismaticJoint1AngleRecord::JOINT1MAXSPEED_MPS;
}

void PrismaticJoint1AngleRecord::disableJoint1MaxSpeed(void)
{
	this->presenceVector &= ~(0x01 << PrismaticJoint1AngleRecord::JOINT1MAXSPEED_MPS);
}

bool PrismaticJoint1AngleRecord::isJoint1MaxForceEnabled(void) const
{
	return (this->presenceVector & (0x01 << PrismaticJoint1AngleRecord::JOINT1MAXFORCE));
}

void PrismaticJoint1AngleRecord::enableJoint1MaxForce(void)
{
	this->presenceVector |= 0x01 << PrismaticJoint1AngleRecord::JOINT1MAXFORCE;
}

void PrismaticJoint1AngleRecord::disableJoint1MaxForce(void)
{
	this->presenceVector &= ~(0x01 << PrismaticJoint1AngleRecord::JOINT1MAXFORCE);
}


void PrismaticJoint1AngleRecord::copy(PrismaticJoint1AngleRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->joint1Angle_rad.setName("Joint1Angle");
	this->joint1Angle_rad.setOptional(false);
	this->joint1Angle_rad.setValue(source.getJoint1Angle_rad()); 
 
	this->joint1MinValue_m.setName("Joint1MinValue");
	this->joint1MinValue_m.setOptional(false);
	this->joint1MinValue_m.setValue(source.getJoint1MinValue_m()); 
 
	this->joint1MaxValue_m.setName("Joint1MaxValue");
	this->joint1MaxValue_m.setOptional(false);
	this->joint1MaxValue_m.setValue(source.getJoint1MaxValue_m()); 
 
	this->joint1MaxSpeed_mps.setName("Joint1MaxSpeed");
	this->joint1MaxSpeed_mps.setOptional(true);
	this->joint1MaxSpeed_mps.setValue(source.getJoint1MaxSpeed_mps()); 
 
	this->joint1MaxForce.setName("Joint1MaxForce");
	this->joint1MaxForce.setOptional(true);
	this->joint1MaxForce.setValue(source.getJoint1MaxForce()); 
 
}

} // namespace manipulator
} // namespace openjaus

