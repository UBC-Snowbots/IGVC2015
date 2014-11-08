/**
\file GlobalWaypointRecord.h

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
#include "openjaus/mobility/Triggers/Fields/GlobalWaypointRecord.h"

namespace openjaus
{
namespace mobility
{

GlobalWaypointRecord::GlobalWaypointRecord():
	latitude_deg(),
	longitude_deg(),
	altitude_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	waypointTolerance_m(),
	pathTolerance()
{
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

GlobalWaypointRecord::GlobalWaypointRecord(const GlobalWaypointRecord &source)
{
	this->copy(const_cast<GlobalWaypointRecord&>(source));
}

GlobalWaypointRecord::~GlobalWaypointRecord()
{

}


double GlobalWaypointRecord::getLatitude_deg(void)
{
	return this->latitude_deg.getValue();
}

void GlobalWaypointRecord::setLatitude_deg(double value)
{
	this->latitude_deg.setValue(value);
}

double GlobalWaypointRecord::getLongitude_deg(void)
{
	return this->longitude_deg.getValue();
}

void GlobalWaypointRecord::setLongitude_deg(double value)
{
	this->longitude_deg.setValue(value);
}

double GlobalWaypointRecord::getAltitude_m(void)
{
	return this->altitude_m.getValue();
}

void GlobalWaypointRecord::setAltitude_m(double value)
{
	this->altitude_m.setValue(value);
}

double GlobalWaypointRecord::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void GlobalWaypointRecord::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double GlobalWaypointRecord::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void GlobalWaypointRecord::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

double GlobalWaypointRecord::getYaw_rad(void)
{
	return this->yaw_rad.getValue();
}

void GlobalWaypointRecord::setYaw_rad(double value)
{
	this->yaw_rad.setValue(value);
}

double GlobalWaypointRecord::getWaypointTolerance_m(void)
{
	return this->waypointTolerance_m.getValue();
}

void GlobalWaypointRecord::setWaypointTolerance_m(double value)
{
	this->waypointTolerance_m.setValue(value);
}

double GlobalWaypointRecord::getPathTolerance(void)
{
	return this->pathTolerance.getValue();
}

void GlobalWaypointRecord::setPathTolerance(double value)
{
	this->pathTolerance.setValue(value);
}

int GlobalWaypointRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
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
int GlobalWaypointRecord::from(system::Buffer *src)
{
	int byteSize = 0;
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

int GlobalWaypointRecord::length(void)
{
	int length = 0;
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

std::string GlobalWaypointRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"GlobalWaypointRecord\">\n";
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
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void GlobalWaypointRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t GlobalWaypointRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool GlobalWaypointRecord::isAltitudeEnabled(void) const
{
	return (this->presenceVector & (0x01 << GlobalWaypointRecord::ALTITUDE_M));
}

void GlobalWaypointRecord::enableAltitude(void)
{
	this->presenceVector |= 0x01 << GlobalWaypointRecord::ALTITUDE_M;
}

void GlobalWaypointRecord::disableAltitude(void)
{
	this->presenceVector &= ~(0x01 << GlobalWaypointRecord::ALTITUDE_M);
}

bool GlobalWaypointRecord::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << GlobalWaypointRecord::ROLL_RAD));
}

void GlobalWaypointRecord::enableRoll(void)
{
	this->presenceVector |= 0x01 << GlobalWaypointRecord::ROLL_RAD;
}

void GlobalWaypointRecord::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << GlobalWaypointRecord::ROLL_RAD);
}

bool GlobalWaypointRecord::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << GlobalWaypointRecord::PITCH_RAD));
}

void GlobalWaypointRecord::enablePitch(void)
{
	this->presenceVector |= 0x01 << GlobalWaypointRecord::PITCH_RAD;
}

void GlobalWaypointRecord::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << GlobalWaypointRecord::PITCH_RAD);
}

bool GlobalWaypointRecord::isYawEnabled(void) const
{
	return (this->presenceVector & (0x01 << GlobalWaypointRecord::YAW_RAD));
}

void GlobalWaypointRecord::enableYaw(void)
{
	this->presenceVector |= 0x01 << GlobalWaypointRecord::YAW_RAD;
}

void GlobalWaypointRecord::disableYaw(void)
{
	this->presenceVector &= ~(0x01 << GlobalWaypointRecord::YAW_RAD);
}

bool GlobalWaypointRecord::isWaypointToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << GlobalWaypointRecord::WAYPOINTTOLERANCE_M));
}

void GlobalWaypointRecord::enableWaypointTolerance(void)
{
	this->presenceVector |= 0x01 << GlobalWaypointRecord::WAYPOINTTOLERANCE_M;
}

void GlobalWaypointRecord::disableWaypointTolerance(void)
{
	this->presenceVector &= ~(0x01 << GlobalWaypointRecord::WAYPOINTTOLERANCE_M);
}

bool GlobalWaypointRecord::isPathToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << GlobalWaypointRecord::PATHTOLERANCE));
}

void GlobalWaypointRecord::enablePathTolerance(void)
{
	this->presenceVector |= 0x01 << GlobalWaypointRecord::PATHTOLERANCE;
}

void GlobalWaypointRecord::disablePathTolerance(void)
{
	this->presenceVector &= ~(0x01 << GlobalWaypointRecord::PATHTOLERANCE);
}


void GlobalWaypointRecord::copy(GlobalWaypointRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->latitude_deg.setName("JausLatitude");
	this->latitude_deg.setOptional(true);
	this->latitude_deg.setValue(source.getLatitude_deg()); 
 
	this->longitude_deg.setName("JausLongitude");
	this->longitude_deg.setOptional(true);
	this->longitude_deg.setValue(source.getLongitude_deg()); 
 
	this->altitude_m.setName("JausAltitude");
	this->altitude_m.setOptional(true);
	this->altitude_m.setValue(source.getAltitude_m()); 
 
	this->roll_rad.setName("Orientation");
	this->roll_rad.setOptional(true);
	this->roll_rad.setValue(source.getRoll_rad()); 
 
	this->pitch_rad.setName("Orientation");
	this->pitch_rad.setOptional(true);
	this->pitch_rad.setValue(source.getPitch_rad()); 
 
	this->yaw_rad.setName("Orientation");
	this->yaw_rad.setOptional(true);
	this->yaw_rad.setValue(source.getYaw_rad()); 
 
	this->waypointTolerance_m.setName("WaypointToleranceRef");
	this->waypointTolerance_m.setOptional(true);
	this->waypointTolerance_m.setValue(source.getWaypointTolerance_m()); 
 
	this->pathTolerance.setName("PathToleranceRef");
	this->pathTolerance.setOptional(true);
	this->pathTolerance.setInterpretation("A value of 0 is used for infinite tolerance.");
	this->pathTolerance.setValue(source.getPathTolerance()); 
 
}

} // namespace mobility
} // namespace openjaus

