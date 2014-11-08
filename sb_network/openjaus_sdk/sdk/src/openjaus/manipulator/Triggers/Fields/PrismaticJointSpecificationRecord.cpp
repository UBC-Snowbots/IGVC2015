/**
\file PrismaticJointSpecificationRecord.h

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
#include "openjaus/manipulator/Triggers/Fields/PrismaticJointSpecificationRecord.h"

namespace openjaus
{
namespace manipulator
{

PrismaticJointSpecificationRecord::PrismaticJointSpecificationRecord():
	linkLength_m(),
	twistAngle_rad(),
	jointAngle_rad(),
	minValue_m(),
	maxValue_m(),
	maxSpeed_mps(),
	maxForce()
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

	fields.push_back(&jointAngle_rad);
	jointAngle_rad.setName("JointAngle");
	jointAngle_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&minValue_m);
	minValue_m.setName("MinValue");
	minValue_m.setOptional(false);
	// Nothing to init

	fields.push_back(&maxValue_m);
	maxValue_m.setName("MaxValue");
	maxValue_m.setOptional(false);
	// Nothing to init

	fields.push_back(&maxSpeed_mps);
	maxSpeed_mps.setName("MaxSpeed");
	maxSpeed_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&maxForce);
	maxForce.setName("MaxForce");
	maxForce.setOptional(true);
	// Nothing to init

}

PrismaticJointSpecificationRecord::PrismaticJointSpecificationRecord(const PrismaticJointSpecificationRecord &source)
{
	this->copy(const_cast<PrismaticJointSpecificationRecord&>(source));
}

PrismaticJointSpecificationRecord::~PrismaticJointSpecificationRecord()
{

}


double PrismaticJointSpecificationRecord::getLinkLength_m(void)
{
	return this->linkLength_m.getValue();
}

void PrismaticJointSpecificationRecord::setLinkLength_m(double value)
{
	this->linkLength_m.setValue(value);
}

double PrismaticJointSpecificationRecord::getTwistAngle_rad(void)
{
	return this->twistAngle_rad.getValue();
}

void PrismaticJointSpecificationRecord::setTwistAngle_rad(double value)
{
	this->twistAngle_rad.setValue(value);
}

double PrismaticJointSpecificationRecord::getJointAngle_rad(void)
{
	return this->jointAngle_rad.getValue();
}

void PrismaticJointSpecificationRecord::setJointAngle_rad(double value)
{
	this->jointAngle_rad.setValue(value);
}

double PrismaticJointSpecificationRecord::getMinValue_m(void)
{
	return this->minValue_m.getValue();
}

void PrismaticJointSpecificationRecord::setMinValue_m(double value)
{
	this->minValue_m.setValue(value);
}

double PrismaticJointSpecificationRecord::getMaxValue_m(void)
{
	return this->maxValue_m.getValue();
}

void PrismaticJointSpecificationRecord::setMaxValue_m(double value)
{
	this->maxValue_m.setValue(value);
}

double PrismaticJointSpecificationRecord::getMaxSpeed_mps(void)
{
	return this->maxSpeed_mps.getValue();
}

void PrismaticJointSpecificationRecord::setMaxSpeed_mps(double value)
{
	this->maxSpeed_mps.setValue(value);
}

double PrismaticJointSpecificationRecord::getMaxForce(void)
{
	return this->maxForce.getValue();
}

void PrismaticJointSpecificationRecord::setMaxForce(double value)
{
	this->maxForce.setValue(value);
}

int PrismaticJointSpecificationRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(linkLength_m);
	byteSize += dst->pack(twistAngle_rad);
	byteSize += dst->pack(jointAngle_rad);
	byteSize += dst->pack(minValue_m);
	byteSize += dst->pack(maxValue_m);
	if(this->isMaxSpeedEnabled())
	{
		byteSize += dst->pack(maxSpeed_mps);
	}
	if(this->isMaxForceEnabled())
	{
		byteSize += dst->pack(maxForce);
	}
	return byteSize;
}
int PrismaticJointSpecificationRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(linkLength_m);
	byteSize += src->unpack(twistAngle_rad);
	byteSize += src->unpack(jointAngle_rad);
	byteSize += src->unpack(minValue_m);
	byteSize += src->unpack(maxValue_m);
	if(this->isMaxSpeedEnabled())
	{
		byteSize += src->unpack(maxSpeed_mps);
	}
	if(this->isMaxForceEnabled())
	{
		byteSize += src->unpack(maxForce);
	}
	return byteSize;
}

int PrismaticJointSpecificationRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += linkLength_m.length(); // linkLength_m
	length += twistAngle_rad.length(); // twistAngle_rad
	length += jointAngle_rad.length(); // jointAngle_rad
	length += minValue_m.length(); // minValue_m
	length += maxValue_m.length(); // maxValue_m
	if(this->isMaxSpeedEnabled())
	{
		length += maxSpeed_mps.length(); // maxSpeed_mps
	}
	if(this->isMaxForceEnabled())
	{
		length += maxForce.length(); // maxForce
	}
	return length;
}

std::string PrismaticJointSpecificationRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"PrismaticJointSpecificationRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaxSpeedEnabled value=\"" << std::boolalpha << this->isMaxSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isMaxForceEnabled value=\"" << std::boolalpha << this->isMaxForceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << linkLength_m.toXml(level+1); // linkLength_m
	oss << twistAngle_rad.toXml(level+1); // twistAngle_rad
	oss << jointAngle_rad.toXml(level+1); // jointAngle_rad
	oss << minValue_m.toXml(level+1); // minValue_m
	oss << maxValue_m.toXml(level+1); // maxValue_m
	if(this->isMaxSpeedEnabled())
	{
		oss << maxSpeed_mps.toXml(level+1); // maxSpeed_mps
	}
	if(this->isMaxForceEnabled())
	{
		oss << maxForce.toXml(level+1); // maxForce
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void PrismaticJointSpecificationRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t PrismaticJointSpecificationRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool PrismaticJointSpecificationRecord::isMaxSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << PrismaticJointSpecificationRecord::MAXSPEED_MPS));
}

void PrismaticJointSpecificationRecord::enableMaxSpeed(void)
{
	this->presenceVector |= 0x01 << PrismaticJointSpecificationRecord::MAXSPEED_MPS;
}

void PrismaticJointSpecificationRecord::disableMaxSpeed(void)
{
	this->presenceVector &= ~(0x01 << PrismaticJointSpecificationRecord::MAXSPEED_MPS);
}

bool PrismaticJointSpecificationRecord::isMaxForceEnabled(void) const
{
	return (this->presenceVector & (0x01 << PrismaticJointSpecificationRecord::MAXFORCE));
}

void PrismaticJointSpecificationRecord::enableMaxForce(void)
{
	this->presenceVector |= 0x01 << PrismaticJointSpecificationRecord::MAXFORCE;
}

void PrismaticJointSpecificationRecord::disableMaxForce(void)
{
	this->presenceVector &= ~(0x01 << PrismaticJointSpecificationRecord::MAXFORCE);
}


void PrismaticJointSpecificationRecord::copy(PrismaticJointSpecificationRecord& source)
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
 
	this->jointAngle_rad.setName("JointAngle");
	this->jointAngle_rad.setOptional(false);
	this->jointAngle_rad.setValue(source.getJointAngle_rad()); 
 
	this->minValue_m.setName("MinValue");
	this->minValue_m.setOptional(false);
	this->minValue_m.setValue(source.getMinValue_m()); 
 
	this->maxValue_m.setName("MaxValue");
	this->maxValue_m.setOptional(false);
	this->maxValue_m.setValue(source.getMaxValue_m()); 
 
	this->maxSpeed_mps.setName("MaxSpeed");
	this->maxSpeed_mps.setOptional(true);
	this->maxSpeed_mps.setValue(source.getMaxSpeed_mps()); 
 
	this->maxForce.setName("MaxForce");
	this->maxForce.setOptional(true);
	this->maxForce.setValue(source.getMaxForce()); 
 
}

} // namespace manipulator
} // namespace openjaus

