
/**
\file ReportLocalPose.h

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
#include "openjaus/mobility/Triggers/ReportLocalPose.h"

namespace openjaus
{
namespace mobility
{

ReportLocalPose::ReportLocalPose() : 
	model::Message(),
	x_m(),
	y_m(),
	z_m(),
	positionRms_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	attitudeRms_rad(),
	timeStamp()
{
	this->id = ReportLocalPose::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&x_m);
	x_m.setName("X");
	x_m.setOptional(true);
	// Nothing to init

	fields.push_back(&y_m);
	y_m.setName("Y");
	y_m.setOptional(true);
	// Nothing to init

	fields.push_back(&z_m);
	z_m.setName("Z");
	z_m.setOptional(true);
	// Nothing to init

	fields.push_back(&positionRms_m);
	positionRms_m.setName("PositionRms");
	positionRms_m.setOptional(true);
	// Nothing to init

	fields.push_back(&roll_rad);
	roll_rad.setName("Roll");
	roll_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&pitch_rad);
	pitch_rad.setName("Pitch");
	pitch_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&yaw_rad);
	yaw_rad.setName("Yaw");
	yaw_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&attitudeRms_rad);
	attitudeRms_rad.setName("AttitudeRms");
	attitudeRms_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(true);
	// Nothing

}

ReportLocalPose::ReportLocalPose(model::Message *message) :
	model::Message(message),
	x_m(),
	y_m(),
	z_m(),
	positionRms_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	attitudeRms_rad(),
	timeStamp()
{
	this->id = ReportLocalPose::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&x_m);
	x_m.setName("X");
	x_m.setOptional(true);
	// Nothing to init

	fields.push_back(&y_m);
	y_m.setName("Y");
	y_m.setOptional(true);
	// Nothing to init

	fields.push_back(&z_m);
	z_m.setName("Z");
	z_m.setOptional(true);
	// Nothing to init

	fields.push_back(&positionRms_m);
	positionRms_m.setName("PositionRms");
	positionRms_m.setOptional(true);
	// Nothing to init

	fields.push_back(&roll_rad);
	roll_rad.setName("Roll");
	roll_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&pitch_rad);
	pitch_rad.setName("Pitch");
	pitch_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&yaw_rad);
	yaw_rad.setName("Yaw");
	yaw_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&attitudeRms_rad);
	attitudeRms_rad.setName("AttitudeRms");
	attitudeRms_rad.setOptional(false);
	// Nothing to init

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(true);
	// Nothing


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportLocalPose::~ReportLocalPose()
{

}


double ReportLocalPose::getX_m(void)
{
	return this->x_m.getValue();
}

void ReportLocalPose::setX_m(double value)
{
	this->x_m.setValue(value);
}

double ReportLocalPose::getY_m(void)
{
	return this->y_m.getValue();
}

void ReportLocalPose::setY_m(double value)
{
	this->y_m.setValue(value);
}

double ReportLocalPose::getZ_m(void)
{
	return this->z_m.getValue();
}

void ReportLocalPose::setZ_m(double value)
{
	this->z_m.setValue(value);
}

double ReportLocalPose::getPositionRms_m(void)
{
	return this->positionRms_m.getValue();
}

void ReportLocalPose::setPositionRms_m(double value)
{
	this->positionRms_m.setValue(value);
}

double ReportLocalPose::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void ReportLocalPose::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double ReportLocalPose::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void ReportLocalPose::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

double ReportLocalPose::getYaw_rad(void)
{
	return this->yaw_rad.getValue();
}

void ReportLocalPose::setYaw_rad(double value)
{
	this->yaw_rad.setValue(value);
}

double ReportLocalPose::getAttitudeRms_rad(void)
{
	return this->attitudeRms_rad.getValue();
}

void ReportLocalPose::setAttitudeRms_rad(double value)
{
	this->attitudeRms_rad.setValue(value);
}

JausTimeStampBitField& ReportLocalPose::getTimeStamp(void)
{
	return this->timeStamp;
}

int ReportLocalPose::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isXEnabled())
	{
		byteSize += dst->pack(x_m);
	}
	if(this->isYEnabled())
	{
		byteSize += dst->pack(y_m);
	}
	if(this->isZEnabled())
	{
		byteSize += dst->pack(z_m);
	}
	if(this->isPositionRmsEnabled())
	{
		byteSize += dst->pack(positionRms_m);
	}
	if(this->isRollEnabled())
	{
		byteSize += dst->pack(roll_rad);
	}
	if(this->isPitchEnabled())
	{
		byteSize += dst->pack(pitch_rad);
	}
	if(this->isYawEnabled())
	{
		byteSize += dst->pack(yaw_rad);
	}
	byteSize += dst->pack(attitudeRms_rad);
	if(this->isTimeStampEnabled())
	{
		byteSize += dst->pack(timeStamp);
	}
	return byteSize;
}

int ReportLocalPose::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isXEnabled())
	{
		byteSize += src->unpack(x_m);
	}
	if(this->isYEnabled())
	{
		byteSize += src->unpack(y_m);
	}
	if(this->isZEnabled())
	{
		byteSize += src->unpack(z_m);
	}
	if(this->isPositionRmsEnabled())
	{
		byteSize += src->unpack(positionRms_m);
	}
	if(this->isRollEnabled())
	{
		byteSize += src->unpack(roll_rad);
	}
	if(this->isPitchEnabled())
	{
		byteSize += src->unpack(pitch_rad);
	}
	if(this->isYawEnabled())
	{
		byteSize += src->unpack(yaw_rad);
	}
	byteSize += src->unpack(attitudeRms_rad);
	if(this->isTimeStampEnabled())
	{
		byteSize += src->unpack(timeStamp);
	}
	return byteSize;
}

int ReportLocalPose::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	if(this->isXEnabled())
	{
		length += x_m.length(); // x_m
	}
	if(this->isYEnabled())
	{
		length += y_m.length(); // y_m
	}
	if(this->isZEnabled())
	{
		length += z_m.length(); // z_m
	}
	if(this->isPositionRmsEnabled())
	{
		length += positionRms_m.length(); // positionRms_m
	}
	if(this->isRollEnabled())
	{
		length += roll_rad.length(); // roll_rad
	}
	if(this->isPitchEnabled())
	{
		length += pitch_rad.length(); // pitch_rad
	}
	if(this->isYawEnabled())
	{
		length += yaw_rad.length(); // yaw_rad
	}
	length += attitudeRms_rad.length(); // attitudeRms_rad
	if(this->isTimeStampEnabled())
	{
		length += timeStamp.length(); // timeStamp
	}
	return length;
}

std::string ReportLocalPose::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportLocalPose\"";
	oss << " id=\"0x4403\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isXEnabled value=\"" << std::boolalpha << this->isXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYEnabled value=\"" << std::boolalpha << this->isYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isZEnabled value=\"" << std::boolalpha << this->isZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPositionRmsEnabled value=\"" << std::boolalpha << this->isPositionRmsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollEnabled value=\"" << std::boolalpha << this->isRollEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchEnabled value=\"" << std::boolalpha << this->isPitchEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawEnabled value=\"" << std::boolalpha << this->isYawEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTimeStampEnabled value=\"" << std::boolalpha << this->isTimeStampEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isXEnabled())
	{
		oss << x_m.toXml(level+1); // x_m
	}
	if(this->isYEnabled())
	{
		oss << y_m.toXml(level+1); // y_m
	}
	if(this->isZEnabled())
	{
		oss << z_m.toXml(level+1); // z_m
	}
	if(this->isPositionRmsEnabled())
	{
		oss << positionRms_m.toXml(level+1); // positionRms_m
	}
	if(this->isRollEnabled())
	{
		oss << roll_rad.toXml(level+1); // roll_rad
	}
	if(this->isPitchEnabled())
	{
		oss << pitch_rad.toXml(level+1); // pitch_rad
	}
	if(this->isYawEnabled())
	{
		oss << yaw_rad.toXml(level+1); // yaw_rad
	}
	oss << attitudeRms_rad.toXml(level+1); // attitudeRms_rad
	if(this->isTimeStampEnabled())
	{
		oss << timeStamp.toXml(level+1); // timeStamp
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportLocalPose::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportLocalPose::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportLocalPose::isXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::X_M));
}

void ReportLocalPose::enableX(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::X_M;
}

void ReportLocalPose::disableX(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::X_M);
}

bool ReportLocalPose::isYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::Y_M));
}

void ReportLocalPose::enableY(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::Y_M;
}

void ReportLocalPose::disableY(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::Y_M);
}

bool ReportLocalPose::isZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::Z_M));
}

void ReportLocalPose::enableZ(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::Z_M;
}

void ReportLocalPose::disableZ(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::Z_M);
}

bool ReportLocalPose::isPositionRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::POSITIONRMS_M));
}

void ReportLocalPose::enablePositionRms(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::POSITIONRMS_M;
}

void ReportLocalPose::disablePositionRms(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::POSITIONRMS_M);
}

bool ReportLocalPose::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::ROLL_RAD));
}

void ReportLocalPose::enableRoll(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::ROLL_RAD;
}

void ReportLocalPose::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::ROLL_RAD);
}

bool ReportLocalPose::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::PITCH_RAD));
}

void ReportLocalPose::enablePitch(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::PITCH_RAD;
}

void ReportLocalPose::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::PITCH_RAD);
}

bool ReportLocalPose::isYawEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::YAW_RAD));
}

void ReportLocalPose::enableYaw(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::YAW_RAD;
}

void ReportLocalPose::disableYaw(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::YAW_RAD);
}

bool ReportLocalPose::isTimeStampEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportLocalPose::TIMESTAMP));
}

void ReportLocalPose::enableTimeStamp(void)
{
	this->presenceVector |= 0x01 << ReportLocalPose::TIMESTAMP;
}

void ReportLocalPose::disableTimeStamp(void)
{
	this->presenceVector &= ~(0x01 << ReportLocalPose::TIMESTAMP);
}

} // namespace mobility
} // namespace openjaus


