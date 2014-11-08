/**
\file LocalVectorRecord.h

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
#include "openjaus/mobility/Triggers/Fields/LocalVectorRecord.h"

namespace openjaus
{
namespace mobility
{

LocalVectorRecord::LocalVectorRecord():
	speed_mps(),
	altitude_m(),
	heading_rad(),
	roll_rad(),
	pitch_rad()
{
	this->presenceVector = 0;

	fields.push_back(&speed_mps);
	speed_mps.setName("Speed");
	speed_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&altitude_m);
	altitude_m.setName("Altitude");
	altitude_m.setOptional(true);
	// Nothing to init

	fields.push_back(&heading_rad);
	heading_rad.setName("Heading");
	heading_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&roll_rad);
	roll_rad.setName("Roll");
	roll_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&pitch_rad);
	pitch_rad.setName("Pitch");
	pitch_rad.setOptional(true);
	// Nothing to init

}

LocalVectorRecord::LocalVectorRecord(const LocalVectorRecord &source)
{
	this->copy(const_cast<LocalVectorRecord&>(source));
}

LocalVectorRecord::~LocalVectorRecord()
{

}


double LocalVectorRecord::getSpeed_mps(void)
{
	return this->speed_mps.getValue();
}

void LocalVectorRecord::setSpeed_mps(double value)
{
	this->speed_mps.setValue(value);
}

double LocalVectorRecord::getAltitude_m(void)
{
	return this->altitude_m.getValue();
}

void LocalVectorRecord::setAltitude_m(double value)
{
	this->altitude_m.setValue(value);
}

double LocalVectorRecord::getHeading_rad(void)
{
	return this->heading_rad.getValue();
}

void LocalVectorRecord::setHeading_rad(double value)
{
	this->heading_rad.setValue(value);
}

double LocalVectorRecord::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void LocalVectorRecord::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double LocalVectorRecord::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void LocalVectorRecord::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

int LocalVectorRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	if(this->isSpeedEnabled())
	{
		byteSize += dst->pack(speed_mps);
	}
	if(this->isAltitudeEnabled())
	{
		byteSize += dst->pack(altitude_m);
	}
	if(this->isHeadingEnabled())
	{
		byteSize += dst->pack(heading_rad);
	}
	if(this->isRollEnabled())
	{
		byteSize += dst->pack(roll_rad);
	}
	if(this->isPitchEnabled())
	{
		byteSize += dst->pack(pitch_rad);
	}
	return byteSize;
}
int LocalVectorRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	if(this->isSpeedEnabled())
	{
		byteSize += src->unpack(speed_mps);
	}
	if(this->isAltitudeEnabled())
	{
		byteSize += src->unpack(altitude_m);
	}
	if(this->isHeadingEnabled())
	{
		byteSize += src->unpack(heading_rad);
	}
	if(this->isRollEnabled())
	{
		byteSize += src->unpack(roll_rad);
	}
	if(this->isPitchEnabled())
	{
		byteSize += src->unpack(pitch_rad);
	}
	return byteSize;
}

int LocalVectorRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	if(this->isSpeedEnabled())
	{
		length += speed_mps.length(); // speed_mps
	}
	if(this->isAltitudeEnabled())
	{
		length += altitude_m.length(); // altitude_m
	}
	if(this->isHeadingEnabled())
	{
		length += heading_rad.length(); // heading_rad
	}
	if(this->isRollEnabled())
	{
		length += roll_rad.length(); // roll_rad
	}
	if(this->isPitchEnabled())
	{
		length += pitch_rad.length(); // pitch_rad
	}
	return length;
}

std::string LocalVectorRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"LocalVectorRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isSpeedEnabled value=\"" << std::boolalpha << this->isSpeedEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAltitudeEnabled value=\"" << std::boolalpha << this->isAltitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isHeadingEnabled value=\"" << std::boolalpha << this->isHeadingEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollEnabled value=\"" << std::boolalpha << this->isRollEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchEnabled value=\"" << std::boolalpha << this->isPitchEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isSpeedEnabled())
	{
		oss << speed_mps.toXml(level+1); // speed_mps
	}
	if(this->isAltitudeEnabled())
	{
		oss << altitude_m.toXml(level+1); // altitude_m
	}
	if(this->isHeadingEnabled())
	{
		oss << heading_rad.toXml(level+1); // heading_rad
	}
	if(this->isRollEnabled())
	{
		oss << roll_rad.toXml(level+1); // roll_rad
	}
	if(this->isPitchEnabled())
	{
		oss << pitch_rad.toXml(level+1); // pitch_rad
	}
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void LocalVectorRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t LocalVectorRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool LocalVectorRecord::isSpeedEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalVectorRecord::SPEED_MPS));
}

void LocalVectorRecord::enableSpeed(void)
{
	this->presenceVector |= 0x01 << LocalVectorRecord::SPEED_MPS;
}

void LocalVectorRecord::disableSpeed(void)
{
	this->presenceVector &= ~(0x01 << LocalVectorRecord::SPEED_MPS);
}

bool LocalVectorRecord::isAltitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalVectorRecord::ALTITUDE_M));
}

void LocalVectorRecord::enableAltitude(void)
{
	this->presenceVector |= 0x01 << LocalVectorRecord::ALTITUDE_M;
}

void LocalVectorRecord::disableAltitude(void)
{
	this->presenceVector &= ~(0x01 << LocalVectorRecord::ALTITUDE_M);
}

bool LocalVectorRecord::isHeadingEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalVectorRecord::HEADING_RAD));
}

void LocalVectorRecord::enableHeading(void)
{
	this->presenceVector |= 0x01 << LocalVectorRecord::HEADING_RAD;
}

void LocalVectorRecord::disableHeading(void)
{
	this->presenceVector &= ~(0x01 << LocalVectorRecord::HEADING_RAD);
}

bool LocalVectorRecord::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalVectorRecord::ROLL_RAD));
}

void LocalVectorRecord::enableRoll(void)
{
	this->presenceVector |= 0x01 << LocalVectorRecord::ROLL_RAD;
}

void LocalVectorRecord::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << LocalVectorRecord::ROLL_RAD);
}

bool LocalVectorRecord::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << LocalVectorRecord::PITCH_RAD));
}

void LocalVectorRecord::enablePitch(void)
{
	this->presenceVector |= 0x01 << LocalVectorRecord::PITCH_RAD;
}

void LocalVectorRecord::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << LocalVectorRecord::PITCH_RAD);
}


void LocalVectorRecord::copy(LocalVectorRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->speed_mps.setName("LinearSpeed");
	this->speed_mps.setOptional(false);
	this->speed_mps.setValue(source.getSpeed_mps()); 
 
	this->altitude_m.setName("JausAltitude");
	this->altitude_m.setOptional(true);
	this->altitude_m.setValue(source.getAltitude_m()); 
 
	this->heading_rad.setName("Orientation");
	this->heading_rad.setOptional(true);
	this->heading_rad.setValue(source.getHeading_rad()); 
 
	this->roll_rad.setName("Orientation");
	this->roll_rad.setOptional(true);
	this->roll_rad.setValue(source.getRoll_rad()); 
 
	this->pitch_rad.setName("Orientation");
	this->pitch_rad.setOptional(true);
	this->pitch_rad.setValue(source.getPitch_rad()); 
 
}

} // namespace mobility
} // namespace openjaus

