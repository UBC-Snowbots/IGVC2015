
/**
\file SetLocalWaypoint.h

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
#include "openjaus/mobility/Triggers/SetLocalWaypoint.h"

namespace openjaus
{
namespace mobility
{

SetLocalWaypoint::SetLocalWaypoint() : 
	model::Message(),
	x_m(),
	y_m(),
	z_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	waypointTolerance_m(),
	pathTolerance()
{
	this->id = SetLocalWaypoint::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&x_m);
	x_m.setName("X");
	x_m.setOptional(false);
	// Nothing to init

	fields.push_back(&y_m);
	y_m.setName("Y");
	y_m.setOptional(false);
	// Nothing to init

	fields.push_back(&z_m);
	z_m.setName("Z");
	z_m.setOptional(true);
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

SetLocalWaypoint::SetLocalWaypoint(model::Message *message) :
	model::Message(message),
	x_m(),
	y_m(),
	z_m(),
	roll_rad(),
	pitch_rad(),
	yaw_rad(),
	waypointTolerance_m(),
	pathTolerance()
{
	this->id = SetLocalWaypoint::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&x_m);
	x_m.setName("X");
	x_m.setOptional(false);
	// Nothing to init

	fields.push_back(&y_m);
	y_m.setName("Y");
	y_m.setOptional(false);
	// Nothing to init

	fields.push_back(&z_m);
	z_m.setName("Z");
	z_m.setOptional(true);
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

SetLocalWaypoint::~SetLocalWaypoint()
{

}


double SetLocalWaypoint::getX_m(void)
{
	return this->x_m.getValue();
}

void SetLocalWaypoint::setX_m(double value)
{
	this->x_m.setValue(value);
}

double SetLocalWaypoint::getY_m(void)
{
	return this->y_m.getValue();
}

void SetLocalWaypoint::setY_m(double value)
{
	this->y_m.setValue(value);
}

double SetLocalWaypoint::getZ_m(void)
{
	return this->z_m.getValue();
}

void SetLocalWaypoint::setZ_m(double value)
{
	this->z_m.setValue(value);
}

double SetLocalWaypoint::getRoll_rad(void)
{
	return this->roll_rad.getValue();
}

void SetLocalWaypoint::setRoll_rad(double value)
{
	this->roll_rad.setValue(value);
}

double SetLocalWaypoint::getPitch_rad(void)
{
	return this->pitch_rad.getValue();
}

void SetLocalWaypoint::setPitch_rad(double value)
{
	this->pitch_rad.setValue(value);
}

double SetLocalWaypoint::getYaw_rad(void)
{
	return this->yaw_rad.getValue();
}

void SetLocalWaypoint::setYaw_rad(double value)
{
	this->yaw_rad.setValue(value);
}

double SetLocalWaypoint::getWaypointTolerance_m(void)
{
	return this->waypointTolerance_m.getValue();
}

void SetLocalWaypoint::setWaypointTolerance_m(double value)
{
	this->waypointTolerance_m.setValue(value);
}

double SetLocalWaypoint::getPathTolerance(void)
{
	return this->pathTolerance.getValue();
}

void SetLocalWaypoint::setPathTolerance(double value)
{
	this->pathTolerance.setValue(value);
}

int SetLocalWaypoint::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(x_m);
	byteSize += dst->pack(y_m);
	if(this->isZEnabled())
	{
		byteSize += dst->pack(z_m);
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

int SetLocalWaypoint::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(x_m);
	byteSize += src->unpack(y_m);
	if(this->isZEnabled())
	{
		byteSize += src->unpack(z_m);
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

int SetLocalWaypoint::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	length += x_m.length(); // x_m
	length += y_m.length(); // y_m
	if(this->isZEnabled())
	{
		length += z_m.length(); // z_m
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

std::string SetLocalWaypoint::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"SetLocalWaypoint\"";
	oss << " id=\"0x040D\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isZEnabled value=\"" << std::boolalpha << this->isZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollEnabled value=\"" << std::boolalpha << this->isRollEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchEnabled value=\"" << std::boolalpha << this->isPitchEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawEnabled value=\"" << std::boolalpha << this->isYawEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isWaypointToleranceEnabled value=\"" << std::boolalpha << this->isWaypointToleranceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPathToleranceEnabled value=\"" << std::boolalpha << this->isPathToleranceEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << x_m.toXml(level+1); // x_m
	oss << y_m.toXml(level+1); // y_m
	if(this->isZEnabled())
	{
		oss << z_m.toXml(level+1); // z_m
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

void SetLocalWaypoint::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t SetLocalWaypoint::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool SetLocalWaypoint::isZEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalWaypoint::Z_M));
}

void SetLocalWaypoint::enableZ(void)
{
	this->presenceVector |= 0x01 << SetLocalWaypoint::Z_M;
}

void SetLocalWaypoint::disableZ(void)
{
	this->presenceVector &= ~(0x01 << SetLocalWaypoint::Z_M);
}

bool SetLocalWaypoint::isRollEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalWaypoint::ROLL_RAD));
}

void SetLocalWaypoint::enableRoll(void)
{
	this->presenceVector |= 0x01 << SetLocalWaypoint::ROLL_RAD;
}

void SetLocalWaypoint::disableRoll(void)
{
	this->presenceVector &= ~(0x01 << SetLocalWaypoint::ROLL_RAD);
}

bool SetLocalWaypoint::isPitchEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalWaypoint::PITCH_RAD));
}

void SetLocalWaypoint::enablePitch(void)
{
	this->presenceVector |= 0x01 << SetLocalWaypoint::PITCH_RAD;
}

void SetLocalWaypoint::disablePitch(void)
{
	this->presenceVector &= ~(0x01 << SetLocalWaypoint::PITCH_RAD);
}

bool SetLocalWaypoint::isYawEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalWaypoint::YAW_RAD));
}

void SetLocalWaypoint::enableYaw(void)
{
	this->presenceVector |= 0x01 << SetLocalWaypoint::YAW_RAD;
}

void SetLocalWaypoint::disableYaw(void)
{
	this->presenceVector &= ~(0x01 << SetLocalWaypoint::YAW_RAD);
}

bool SetLocalWaypoint::isWaypointToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalWaypoint::WAYPOINTTOLERANCE_M));
}

void SetLocalWaypoint::enableWaypointTolerance(void)
{
	this->presenceVector |= 0x01 << SetLocalWaypoint::WAYPOINTTOLERANCE_M;
}

void SetLocalWaypoint::disableWaypointTolerance(void)
{
	this->presenceVector &= ~(0x01 << SetLocalWaypoint::WAYPOINTTOLERANCE_M);
}

bool SetLocalWaypoint::isPathToleranceEnabled(void) const
{
	return (this->presenceVector & (0x01 << SetLocalWaypoint::PATHTOLERANCE));
}

void SetLocalWaypoint::enablePathTolerance(void)
{
	this->presenceVector |= 0x01 << SetLocalWaypoint::PATHTOLERANCE;
}

void SetLocalWaypoint::disablePathTolerance(void)
{
	this->presenceVector &= ~(0x01 << SetLocalWaypoint::PATHTOLERANCE);
}

} // namespace mobility
} // namespace openjaus


