
/**
\file SetLocalPose.h

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
#include "openjaus/mobility/Triggers/SetLocalPose.h"

namespace openjaus
{
namespace mobility
{

SetLocalPose::SetLocalPose() : 
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
	this->id = SetLocalPose::ID; // Initialize id member
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

SetLocalPose::SetLocalPose(model::Message *message) :
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
	this->id = SetLocalPose::ID; // Initialize id member
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

SetLocalPose::~SetLocalPose()
{

}


double SetLocalPose::getX_m(void)
{
	return this->x_m.getValue();
}

void SetLocalPose::setX_m(double value)
{
	this->x_m.setValue(value);
}

double SetLocalPose::getY_m(void)
{
	return this->y_m.getValue();
}

void SetLocalPose::setY_m(double value)
{
	this->y_m.setValue(value);
}

double SetLocalPose::getZ_m(void)
{
	return this->z_m.getValue();
}

void SetLocalPose::setZ_m(double value)
{
	this->z_m.setValue(value);
}

double SetLocalPose::getPositionRms_m(void)
{
	return this->positionRms_m.getValue();
}

void SetLocalPose::setPositionRms_m(double value)
{
	this->positionRms_m.setValue(value);
}

double SetLocalPose::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void SetLocalPose::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double SetLocalPose::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void SetLocalPose::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

double SetLocalPose::getYaw_rad(void)
{
	return this->yaw_rad.getValue();
}

void SetLocalPose::setYaw_rad(double value)
{
	this->yaw_rad.setValue(value);
}

double SetLocalPose::getAttitudeRms_rad(void)
{
	return this->attitudeRms_rad.getValue();
}

void SetLocalPose::setAttitudeRms_rad(double value)
{
	this->attitudeRms_rad.setValue(value);
}

JausTimeStampBitField& SetLocalPose::getTimeStamp(void)
{
	return this->timeStamp;
}

int SetLocalPose::to(system::Buffer *dst)
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

int SetLocalPose::from(system::Buffer *src)
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

int SetLocalPose::length(void)
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

std::string SetLocalPose::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetLocalPose\"";
	oss << " id=\"0x0403\" >\n";
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

void SetLocalPose::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t SetLocalPose::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool SetLocalPose::isXEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::X_M));
}

void SetLocalPose::enableX(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::X_M;
}

void SetLocalPose::disableX(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::X_M);
}

bool SetLocalPose::isYEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::Y_M));
}

void SetLocalPose::enableY(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::Y_M;
}

void SetLocalPose::disableY(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::Y_M);
}

bool SetLocalPose::isZEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::Z_M));
}

void SetLocalPose::enableZ(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::Z_M;
}

void SetLocalPose::disableZ(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::Z_M);
}

bool SetLocalPose::isPositionRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::POSITIONRMS_M));
}

void SetLocalPose::enablePositionRms(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::POSITIONRMS_M;
}

void SetLocalPose::disablePositionRms(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::POSITIONRMS_M);
}

bool SetLocalPose::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::ROLL_RAD));
}

void SetLocalPose::enableRoll(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::ROLL_RAD;
}

void SetLocalPose::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::ROLL_RAD);
}

bool SetLocalPose::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::PITCH_RAD));
}

void SetLocalPose::enablePitch(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::PITCH_RAD;
}

void SetLocalPose::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::PITCH_RAD);
}

bool SetLocalPose::isYawEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::YAW_RAD));
}

void SetLocalPose::enableYaw(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::YAW_RAD;
}

void SetLocalPose::disableYaw(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::YAW_RAD);
}

bool SetLocalPose::isTimeStampEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalPose::TIMESTAMP));
}

void SetLocalPose::enableTimeStamp(void)
{
	this->presenceVector |= 0x01 << SetLocalPose::TIMESTAMP;
}

void SetLocalPose::disableTimeStamp(void)
{
	this->presenceVector &= ~(0x01 << SetLocalPose::TIMESTAMP);
}

} // namespace mobility
} // namespace openjaus


