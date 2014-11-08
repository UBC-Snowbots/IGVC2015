
/**
\file ReportGlobalPose.h

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
#include "openjaus/mobility/Triggers/ReportGlobalPose.h"

namespace openjaus
{
namespace mobility
{

ReportGlobalPose::ReportGlobalPose() : 
	model::Message(),
	latitude_deg(),
	longitude_deg(),
	altitude_m(),
	positionRms_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	attitudeRms_rad(),
	timeStamp()
{
	this->id = ReportGlobalPose::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&latitude_deg);
	latitude_deg.setName("Latitude");
	latitude_deg.setOptional(true);
	// Nothing to init

	fields.push_back(&longitude_deg);
	longitude_deg.setName("Longitude");
	longitude_deg.setOptional(true);
	// Nothing to init

	fields.push_back(&altitude_m);
	altitude_m.setName("Altitude");
	altitude_m.setOptional(true);
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
	attitudeRms_rad.setOptional(true);
	// Nothing to init

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(true);
	// Nothing

}

ReportGlobalPose::ReportGlobalPose(model::Message *message) :
	model::Message(message),
	latitude_deg(),
	longitude_deg(),
	altitude_m(),
	positionRms_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	attitudeRms_rad(),
	timeStamp()
{
	this->id = ReportGlobalPose::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&latitude_deg);
	latitude_deg.setName("Latitude");
	latitude_deg.setOptional(true);
	// Nothing to init

	fields.push_back(&longitude_deg);
	longitude_deg.setName("Longitude");
	longitude_deg.setOptional(true);
	// Nothing to init

	fields.push_back(&altitude_m);
	altitude_m.setName("Altitude");
	altitude_m.setOptional(true);
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
	attitudeRms_rad.setOptional(true);
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

ReportGlobalPose::~ReportGlobalPose()
{

}


double ReportGlobalPose::getLatitude_deg(void)
{
	return this->latitude_deg.getValue();
}

void ReportGlobalPose::setLatitude_deg(double value)
{
	this->latitude_deg.setValue(value);
}

double ReportGlobalPose::getLongitude_deg(void)
{
	return this->longitude_deg.getValue();
}

void ReportGlobalPose::setLongitude_deg(double value)
{
	this->longitude_deg.setValue(value);
}

double ReportGlobalPose::getAltitude_m(void)
{
	return this->altitude_m.getValue();
}

void ReportGlobalPose::setAltitude_m(double value)
{
	this->altitude_m.setValue(value);
}

double ReportGlobalPose::getPositionRms_m(void)
{
	return this->positionRms_m.getValue();
}

void ReportGlobalPose::setPositionRms_m(double value)
{
	this->positionRms_m.setValue(value);
}

double ReportGlobalPose::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void ReportGlobalPose::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double ReportGlobalPose::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void ReportGlobalPose::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

double ReportGlobalPose::getYaw_rad(void)
{
	return this->yaw_rad.getValue();
}

void ReportGlobalPose::setYaw_rad(double value)
{
	this->yaw_rad.setValue(value);
}

double ReportGlobalPose::getAttitudeRms_rad(void)
{
	return this->attitudeRms_rad.getValue();
}

void ReportGlobalPose::setAttitudeRms_rad(double value)
{
	this->attitudeRms_rad.setValue(value);
}

JausTimeStampBitField& ReportGlobalPose::getTimeStamp(void)
{
	return this->timeStamp;
}

int ReportGlobalPose::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isLatitudeEnabled())
	{
		byteSize += dst->pack(latitude_deg);
	}
	if(this->isLongitudeEnabled())
	{
		byteSize += dst->pack(longitude_deg);
	}
	if(this->isAltitudeEnabled())
	{
		byteSize += dst->pack(altitude_m);
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
	if(this->isAttitudeRmsEnabled())
	{
		byteSize += dst->pack(attitudeRms_rad);
	}
	if(this->isTimeStampEnabled())
	{
		byteSize += dst->pack(timeStamp);
	}
	return byteSize;
}

int ReportGlobalPose::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isLatitudeEnabled())
	{
		byteSize += src->unpack(latitude_deg);
	}
	if(this->isLongitudeEnabled())
	{
		byteSize += src->unpack(longitude_deg);
	}
	if(this->isAltitudeEnabled())
	{
		byteSize += src->unpack(altitude_m);
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
	if(this->isAttitudeRmsEnabled())
	{
		byteSize += src->unpack(attitudeRms_rad);
	}
	if(this->isTimeStampEnabled())
	{
		byteSize += src->unpack(timeStamp);
	}
	return byteSize;
}

int ReportGlobalPose::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint16_t); // PresenceVector
	if(this->isLatitudeEnabled())
	{
		length += latitude_deg.length(); // latitude_deg
	}
	if(this->isLongitudeEnabled())
	{
		length += longitude_deg.length(); // longitude_deg
	}
	if(this->isAltitudeEnabled())
	{
		length += altitude_m.length(); // altitude_m
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
	if(this->isAttitudeRmsEnabled())
	{
		length += attitudeRms_rad.length(); // attitudeRms_rad
	}
	if(this->isTimeStampEnabled())
	{
		length += timeStamp.length(); // timeStamp
	}
	return length;
}

std::string ReportGlobalPose::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportGlobalPose\"";
	oss << " id=\"0x4402\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isLatitudeEnabled value=\"" << std::boolalpha << this->isLatitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isLongitudeEnabled value=\"" << std::boolalpha << this->isLongitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAltitudeEnabled value=\"" << std::boolalpha << this->isAltitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPositionRmsEnabled value=\"" << std::boolalpha << this->isPositionRmsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollEnabled value=\"" << std::boolalpha << this->isRollEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchEnabled value=\"" << std::boolalpha << this->isPitchEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawEnabled value=\"" << std::boolalpha << this->isYawEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAttitudeRmsEnabled value=\"" << std::boolalpha << this->isAttitudeRmsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTimeStampEnabled value=\"" << std::boolalpha << this->isTimeStampEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isLatitudeEnabled())
	{
		oss << latitude_deg.toXml(level+1); // latitude_deg
	}
	if(this->isLongitudeEnabled())
	{
		oss << longitude_deg.toXml(level+1); // longitude_deg
	}
	if(this->isAltitudeEnabled())
	{
		oss << altitude_m.toXml(level+1); // altitude_m
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
	if(this->isAttitudeRmsEnabled())
	{
		oss << attitudeRms_rad.toXml(level+1); // attitudeRms_rad
	}
	if(this->isTimeStampEnabled())
	{
		oss << timeStamp.toXml(level+1); // timeStamp
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportGlobalPose::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t ReportGlobalPose::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportGlobalPose::isLatitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::LATITUDE_DEG));
}

void ReportGlobalPose::enableLatitude(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::LATITUDE_DEG;
}

void ReportGlobalPose::disableLatitude(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::LATITUDE_DEG);
}

bool ReportGlobalPose::isLongitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::LONGITUDE_DEG));
}

void ReportGlobalPose::enableLongitude(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::LONGITUDE_DEG;
}

void ReportGlobalPose::disableLongitude(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::LONGITUDE_DEG);
}

bool ReportGlobalPose::isAltitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::ALTITUDE_M));
}

void ReportGlobalPose::enableAltitude(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::ALTITUDE_M;
}

void ReportGlobalPose::disableAltitude(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::ALTITUDE_M);
}

bool ReportGlobalPose::isPositionRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::POSITIONRMS_M));
}

void ReportGlobalPose::enablePositionRms(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::POSITIONRMS_M;
}

void ReportGlobalPose::disablePositionRms(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::POSITIONRMS_M);
}

bool ReportGlobalPose::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::ROLL_RAD));
}

void ReportGlobalPose::enableRoll(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::ROLL_RAD;
}

void ReportGlobalPose::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::ROLL_RAD);
}

bool ReportGlobalPose::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::PITCH_RAD));
}

void ReportGlobalPose::enablePitch(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::PITCH_RAD;
}

void ReportGlobalPose::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::PITCH_RAD);
}

bool ReportGlobalPose::isYawEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::YAW_RAD));
}

void ReportGlobalPose::enableYaw(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::YAW_RAD;
}

void ReportGlobalPose::disableYaw(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::YAW_RAD);
}

bool ReportGlobalPose::isAttitudeRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::ATTITUDERMS_RAD));
}

void ReportGlobalPose::enableAttitudeRms(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::ATTITUDERMS_RAD;
}

void ReportGlobalPose::disableAttitudeRms(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::ATTITUDERMS_RAD);
}

bool ReportGlobalPose::isTimeStampEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalPose::TIMESTAMP));
}

void ReportGlobalPose::enableTimeStamp(void)
{
	this->presenceVector |= 0x01 << ReportGlobalPose::TIMESTAMP;
}

void ReportGlobalPose::disableTimeStamp(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalPose::TIMESTAMP);
}

} // namespace mobility
} // namespace openjaus


