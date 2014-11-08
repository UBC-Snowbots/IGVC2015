
/**
\file ReportGlobalWaypoint.h

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
#include "openjaus/mobility/Triggers/ReportGlobalWaypoint.h"

namespace openjaus
{
namespace mobility
{

ReportGlobalWaypoint::ReportGlobalWaypoint() : 
	model::Message(),
	latitude_deg(),
	longitude_deg(),
	altitude_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	waypointTolerance_m(),
	pathTolerance()
{
	this->id = ReportGlobalWaypoint::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&latitude_deg);
	latitude_deg.setName("Latitude");
	latitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&longitude_deg);
	longitude_deg.setName("Longitude");
	longitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&altitude_m);
	altitude_m.setName("Altitude");
	altitude_m.setOptional(true);
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

	fields.push_back(&waypointTolerance_m);
	waypointTolerance_m.setName("WaypointTolerance");
	waypointTolerance_m.setOptional(true);
	// Nothing to init

	fields.push_back(&pathTolerance);
	pathTolerance.setName("PathTolerance");
	pathTolerance.setOptional(true);
	pathTolerance.setInterpretation("A value of 0 is used for infinite tolerance.");
	// Nothing to init

}

ReportGlobalWaypoint::ReportGlobalWaypoint(model::Message *message) :
	model::Message(message),
	latitude_deg(),
	longitude_deg(),
	altitude_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	waypointTolerance_m(),
	pathTolerance()
{
	this->id = ReportGlobalWaypoint::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&latitude_deg);
	latitude_deg.setName("Latitude");
	latitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&longitude_deg);
	longitude_deg.setName("Longitude");
	longitude_deg.setOptional(false);
	// Nothing to init

	fields.push_back(&altitude_m);
	altitude_m.setName("Altitude");
	altitude_m.setOptional(true);
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

	fields.push_back(&waypointTolerance_m);
	waypointTolerance_m.setName("WaypointTolerance");
	waypointTolerance_m.setOptional(true);
	// Nothing to init

	fields.push_back(&pathTolerance);
	pathTolerance.setName("PathTolerance");
	pathTolerance.setOptional(true);
	pathTolerance.setInterpretation("A value of 0 is used for infinite tolerance.");
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportGlobalWaypoint::~ReportGlobalWaypoint()
{

}


double ReportGlobalWaypoint::getLatitude_deg(void)
{
	return this->latitude_deg.getValue();
}

void ReportGlobalWaypoint::setLatitude_deg(double value)
{
	this->latitude_deg.setValue(value);
}

double ReportGlobalWaypoint::getLongitude_deg(void)
{
	return this->longitude_deg.getValue();
}

void ReportGlobalWaypoint::setLongitude_deg(double value)
{
	this->longitude_deg.setValue(value);
}

double ReportGlobalWaypoint::getAltitude_m(void)
{
	return this->altitude_m.getValue();
}

void ReportGlobalWaypoint::setAltitude_m(double value)
{
	this->altitude_m.setValue(value);
}

double ReportGlobalWaypoint::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void ReportGlobalWaypoint::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double ReportGlobalWaypoint::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void ReportGlobalWaypoint::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

double ReportGlobalWaypoint::getYaw_rad(void)
{
	return this->yaw_rad.getValue();
}

void ReportGlobalWaypoint::setYaw_rad(double value)
{
	this->yaw_rad.setValue(value);
}

double ReportGlobalWaypoint::getWaypointTolerance_m(void)
{
	return this->waypointTolerance_m.getValue();
}

void ReportGlobalWaypoint::setWaypointTolerance_m(double value)
{
	this->waypointTolerance_m.setValue(value);
}

double ReportGlobalWaypoint::getPathTolerance(void)
{
	return this->pathTolerance.getValue();
}

void ReportGlobalWaypoint::setPathTolerance(double value)
{
	this->pathTolerance.setValue(value);
}

int ReportGlobalWaypoint::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(latitude_deg);
	byteSize += dst->pack(longitude_deg);
	if(this->isAltitudeEnabled())
	{
		byteSize += dst->pack(altitude_m);
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
	if(this->isWaypointToleranceEnabled())
	{
		byteSize += dst->pack(waypointTolerance_m);
	}
	if(this->isPathToleranceEnabled())
	{
		byteSize += dst->pack(pathTolerance);
	}
	return byteSize;
}

int ReportGlobalWaypoint::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(latitude_deg);
	byteSize += src->unpack(longitude_deg);
	if(this->isAltitudeEnabled())
	{
		byteSize += src->unpack(altitude_m);
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
	if(this->isWaypointToleranceEnabled())
	{
		byteSize += src->unpack(waypointTolerance_m);
	}
	if(this->isPathToleranceEnabled())
	{
		byteSize += src->unpack(pathTolerance);
	}
	return byteSize;
}

int ReportGlobalWaypoint::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	length += latitude_deg.length(); // latitude_deg
	length += longitude_deg.length(); // longitude_deg
	if(this->isAltitudeEnabled())
	{
		length += altitude_m.length(); // altitude_m
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
	if(this->isWaypointToleranceEnabled())
	{
		length += waypointTolerance_m.length(); // waypointTolerance_m
	}
	if(this->isPathToleranceEnabled())
	{
		length += pathTolerance.length(); // pathTolerance
	}
	return length;
}

std::string ReportGlobalWaypoint::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportGlobalWaypoint\"";
	oss << " id=\"0x440C\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAltitudeEnabled value=\"" << std::boolalpha << this->isAltitudeEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollEnabled value=\"" << std::boolalpha << this->isRollEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchEnabled value=\"" << std::boolalpha << this->isPitchEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawEnabled value=\"" << std::boolalpha << this->isYawEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isWaypointToleranceEnabled value=\"" << std::boolalpha << this->isWaypointToleranceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPathToleranceEnabled value=\"" << std::boolalpha << this->isPathToleranceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << latitude_deg.toXml(level+1); // latitude_deg
	oss << longitude_deg.toXml(level+1); // longitude_deg
	if(this->isAltitudeEnabled())
	{
		oss << altitude_m.toXml(level+1); // altitude_m
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
	if(this->isWaypointToleranceEnabled())
	{
		oss << waypointTolerance_m.toXml(level+1); // waypointTolerance_m
	}
	if(this->isPathToleranceEnabled())
	{
		oss << pathTolerance.toXml(level+1); // pathTolerance
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportGlobalWaypoint::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportGlobalWaypoint::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportGlobalWaypoint::isAltitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalWaypoint::ALTITUDE_M));
}

void ReportGlobalWaypoint::enableAltitude(void)
{
	this->presenceVector |= 0x01 << ReportGlobalWaypoint::ALTITUDE_M;
}

void ReportGlobalWaypoint::disableAltitude(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalWaypoint::ALTITUDE_M);
}

bool ReportGlobalWaypoint::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalWaypoint::ROLL_RAD));
}

void ReportGlobalWaypoint::enableRoll(void)
{
	this->presenceVector |= 0x01 << ReportGlobalWaypoint::ROLL_RAD;
}

void ReportGlobalWaypoint::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalWaypoint::ROLL_RAD);
}

bool ReportGlobalWaypoint::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalWaypoint::PITCH_RAD));
}

void ReportGlobalWaypoint::enablePitch(void)
{
	this->presenceVector |= 0x01 << ReportGlobalWaypoint::PITCH_RAD;
}

void ReportGlobalWaypoint::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalWaypoint::PITCH_RAD);
}

bool ReportGlobalWaypoint::isYawEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalWaypoint::YAW_RAD));
}

void ReportGlobalWaypoint::enableYaw(void)
{
	this->presenceVector |= 0x01 << ReportGlobalWaypoint::YAW_RAD;
}

void ReportGlobalWaypoint::disableYaw(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalWaypoint::YAW_RAD);
}

bool ReportGlobalWaypoint::isWaypointToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalWaypoint::WAYPOINTTOLERANCE_M));
}

void ReportGlobalWaypoint::enableWaypointTolerance(void)
{
	this->presenceVector |= 0x01 << ReportGlobalWaypoint::WAYPOINTTOLERANCE_M;
}

void ReportGlobalWaypoint::disableWaypointTolerance(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalWaypoint::WAYPOINTTOLERANCE_M);
}

bool ReportGlobalWaypoint::isPathToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportGlobalWaypoint::PATHTOLERANCE));
}

void ReportGlobalWaypoint::enablePathTolerance(void)
{
	this->presenceVector |= 0x01 << ReportGlobalWaypoint::PATHTOLERANCE;
}

void ReportGlobalWaypoint::disablePathTolerance(void)
{
	this->presenceVector &= ~(0x01 << ReportGlobalWaypoint::PATHTOLERANCE);
}

} // namespace mobility
} // namespace openjaus


