
/**
\file ReportAccelerationLimit.h

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
#include "openjaus/mobility/Triggers/ReportAccelerationLimit.h"

namespace openjaus
{
namespace mobility
{

ReportAccelerationLimit::ReportAccelerationLimit() : 
	model::Message(),
	commandType(),
	accelerationX_mps2(),
	accelerationY_mps2(),
	accelerationZ_mps2(),
	rollAcceleration_rps2(),
	pitchAcceleration_rps2(),
	yawAcceleration_rps2()
{
	this->id = ReportAccelerationLimit::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&commandType);
	commandType.setName("CommandType");
	commandType.setOptional(false);
	// Nothing to init

	fields.push_back(&accelerationX_mps2);
	accelerationX_mps2.setName("AccelerationX");
	accelerationX_mps2.setOptional(true);
	// Nothing to init

	fields.push_back(&accelerationY_mps2);
	accelerationY_mps2.setName("AccelerationY");
	accelerationY_mps2.setOptional(true);
	// Nothing to init

	fields.push_back(&accelerationZ_mps2);
	accelerationZ_mps2.setName("AccelerationZ");
	accelerationZ_mps2.setOptional(true);
	// Nothing to init

	fields.push_back(&rollAcceleration_rps2);
	rollAcceleration_rps2.setName("RollAcceleration");
	rollAcceleration_rps2.setOptional(true);
	// Nothing to init

	fields.push_back(&pitchAcceleration_rps2);
	pitchAcceleration_rps2.setName("PitchAcceleration");
	pitchAcceleration_rps2.setOptional(true);
	// Nothing to init

	fields.push_back(&yawAcceleration_rps2);
	yawAcceleration_rps2.setName("YawAcceleration");
	yawAcceleration_rps2.setOptional(true);
	// Nothing to init

}

ReportAccelerationLimit::ReportAccelerationLimit(model::Message *message) :
	model::Message(message),
	commandType(),
	accelerationX_mps2(),
	accelerationY_mps2(),
	accelerationZ_mps2(),
	rollAcceleration_rps2(),
	pitchAcceleration_rps2(),
	yawAcceleration_rps2()
{
	this->id = ReportAccelerationLimit::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&commandType);
	commandType.setName("CommandType");
	commandType.setOptional(false);
	// Nothing to init

	fields.push_back(&accelerationX_mps2);
	accelerationX_mps2.setName("AccelerationX");
	accelerationX_mps2.setOptional(true);
	// Nothing to init

	fields.push_back(&accelerationY_mps2);
	accelerationY_mps2.setName("AccelerationY");
	accelerationY_mps2.setOptional(true);
	// Nothing to init

	fields.push_back(&accelerationZ_mps2);
	accelerationZ_mps2.setName("AccelerationZ");
	accelerationZ_mps2.setOptional(true);
	// Nothing to init

	fields.push_back(&rollAcceleration_rps2);
	rollAcceleration_rps2.setName("RollAcceleration");
	rollAcceleration_rps2.setOptional(true);
	// Nothing to init

	fields.push_back(&pitchAcceleration_rps2);
	pitchAcceleration_rps2.setName("PitchAcceleration");
	pitchAcceleration_rps2.setOptional(true);
	// Nothing to init

	fields.push_back(&yawAcceleration_rps2);
	yawAcceleration_rps2.setName("YawAcceleration");
	yawAcceleration_rps2.setOptional(true);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportAccelerationLimit::~ReportAccelerationLimit()
{

}


AccelerationCommandTypeEnumeration::AccelerationCommandTypeEnum ReportAccelerationLimit::getCommandType(void)
{
	return this->commandType.getValue();
}

void ReportAccelerationLimit::setCommandType(AccelerationCommandTypeEnumeration::AccelerationCommandTypeEnum value)
{
	this->commandType.setValue(value);
}

double ReportAccelerationLimit::getAccelerationX_mps2(void)
{
	return this->accelerationX_mps2.getValue();
}

void ReportAccelerationLimit::setAccelerationX_mps2(double value)
{
	this->accelerationX_mps2.setValue(value);
}

double ReportAccelerationLimit::getAccelerationY_mps2(void)
{
	return this->accelerationY_mps2.getValue();
}

void ReportAccelerationLimit::setAccelerationY_mps2(double value)
{
	this->accelerationY_mps2.setValue(value);
}

double ReportAccelerationLimit::getAccelerationZ_mps2(void)
{
	return this->accelerationZ_mps2.getValue();
}

void ReportAccelerationLimit::setAccelerationZ_mps2(double value)
{
	this->accelerationZ_mps2.setValue(value);
}

double ReportAccelerationLimit::getRollAcceleration_rps2(void)
{
	return this->rollAcceleration_rps2.getValue();
}

void ReportAccelerationLimit::setRollAcceleration_rps2(double value)
{
	this->rollAcceleration_rps2.setValue(value);
}

double ReportAccelerationLimit::getPitchAcceleration_rps2(void)
{
	return this->pitchAcceleration_rps2.getValue();
}

void ReportAccelerationLimit::setPitchAcceleration_rps2(double value)
{
	this->pitchAcceleration_rps2.setValue(value);
}

double ReportAccelerationLimit::getYawAcceleration_rps2(void)
{
	return this->yawAcceleration_rps2.getValue();
}

void ReportAccelerationLimit::setYawAcceleration_rps2(double value)
{
	this->yawAcceleration_rps2.setValue(value);
}

int ReportAccelerationLimit::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(commandType);
	if(this->isAccelerationXEnabled())
	{
		byteSize += dst->pack(accelerationX_mps2);
	}
	if(this->isAccelerationYEnabled())
	{
		byteSize += dst->pack(accelerationY_mps2);
	}
	if(this->isAccelerationZEnabled())
	{
		byteSize += dst->pack(accelerationZ_mps2);
	}
	if(this->isRollAccelerationEnabled())
	{
		byteSize += dst->pack(rollAcceleration_rps2);
	}
	if(this->isPitchAccelerationEnabled())
	{
		byteSize += dst->pack(pitchAcceleration_rps2);
	}
	if(this->isYawAccelerationEnabled())
	{
		byteSize += dst->pack(yawAcceleration_rps2);
	}
	return byteSize;
}

int ReportAccelerationLimit::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(commandType);
	if(this->isAccelerationXEnabled())
	{
		byteSize += src->unpack(accelerationX_mps2);
	}
	if(this->isAccelerationYEnabled())
	{
		byteSize += src->unpack(accelerationY_mps2);
	}
	if(this->isAccelerationZEnabled())
	{
		byteSize += src->unpack(accelerationZ_mps2);
	}
	if(this->isRollAccelerationEnabled())
	{
		byteSize += src->unpack(rollAcceleration_rps2);
	}
	if(this->isPitchAccelerationEnabled())
	{
		byteSize += src->unpack(pitchAcceleration_rps2);
	}
	if(this->isYawAccelerationEnabled())
	{
		byteSize += src->unpack(yawAcceleration_rps2);
	}
	return byteSize;
}

int ReportAccelerationLimit::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	length += commandType.length(); // commandType
	if(this->isAccelerationXEnabled())
	{
		length += accelerationX_mps2.length(); // accelerationX_mps2
	}
	if(this->isAccelerationYEnabled())
	{
		length += accelerationY_mps2.length(); // accelerationY_mps2
	}
	if(this->isAccelerationZEnabled())
	{
		length += accelerationZ_mps2.length(); // accelerationZ_mps2
	}
	if(this->isRollAccelerationEnabled())
	{
		length += rollAcceleration_rps2.length(); // rollAcceleration_rps2
	}
	if(this->isPitchAccelerationEnabled())
	{
		length += pitchAcceleration_rps2.length(); // pitchAcceleration_rps2
	}
	if(this->isYawAccelerationEnabled())
	{
		length += yawAcceleration_rps2.length(); // yawAcceleration_rps2
	}
	return length;
}

std::string ReportAccelerationLimit::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportAccelerationLimit\"";
	oss << " id=\"0x4416\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationXEnabled value=\"" << std::boolalpha << this->isAccelerationXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationYEnabled value=\"" << std::boolalpha << this->isAccelerationYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationZEnabled value=\"" << std::boolalpha << this->isAccelerationZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollAccelerationEnabled value=\"" << std::boolalpha << this->isRollAccelerationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchAccelerationEnabled value=\"" << std::boolalpha << this->isPitchAccelerationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawAccelerationEnabled value=\"" << std::boolalpha << this->isYawAccelerationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << commandType.toXml(level+1); // commandType
	if(this->isAccelerationXEnabled())
	{
		oss << accelerationX_mps2.toXml(level+1); // accelerationX_mps2
	}
	if(this->isAccelerationYEnabled())
	{
		oss << accelerationY_mps2.toXml(level+1); // accelerationY_mps2
	}
	if(this->isAccelerationZEnabled())
	{
		oss << accelerationZ_mps2.toXml(level+1); // accelerationZ_mps2
	}
	if(this->isRollAccelerationEnabled())
	{
		oss << rollAcceleration_rps2.toXml(level+1); // rollAcceleration_rps2
	}
	if(this->isPitchAccelerationEnabled())
	{
		oss << pitchAcceleration_rps2.toXml(level+1); // pitchAcceleration_rps2
	}
	if(this->isYawAccelerationEnabled())
	{
		oss << yawAcceleration_rps2.toXml(level+1); // yawAcceleration_rps2
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportAccelerationLimit::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportAccelerationLimit::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportAccelerationLimit::isAccelerationXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationLimit::ACCELERATIONX_MPS2));
}

void ReportAccelerationLimit::enableAccelerationX(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationLimit::ACCELERATIONX_MPS2;
}

void ReportAccelerationLimit::disableAccelerationX(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationLimit::ACCELERATIONX_MPS2);
}

bool ReportAccelerationLimit::isAccelerationYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationLimit::ACCELERATIONY_MPS2));
}

void ReportAccelerationLimit::enableAccelerationY(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationLimit::ACCELERATIONY_MPS2;
}

void ReportAccelerationLimit::disableAccelerationY(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationLimit::ACCELERATIONY_MPS2);
}

bool ReportAccelerationLimit::isAccelerationZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationLimit::ACCELERATIONZ_MPS2));
}

void ReportAccelerationLimit::enableAccelerationZ(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationLimit::ACCELERATIONZ_MPS2;
}

void ReportAccelerationLimit::disableAccelerationZ(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationLimit::ACCELERATIONZ_MPS2);
}

bool ReportAccelerationLimit::isRollAccelerationEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationLimit::ROLLACCELERATION_RPS2));
}

void ReportAccelerationLimit::enableRollAcceleration(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationLimit::ROLLACCELERATION_RPS2;
}

void ReportAccelerationLimit::disableRollAcceleration(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationLimit::ROLLACCELERATION_RPS2);
}

bool ReportAccelerationLimit::isPitchAccelerationEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationLimit::PITCHACCELERATION_RPS2));
}

void ReportAccelerationLimit::enablePitchAcceleration(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationLimit::PITCHACCELERATION_RPS2;
}

void ReportAccelerationLimit::disablePitchAcceleration(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationLimit::PITCHACCELERATION_RPS2);
}

bool ReportAccelerationLimit::isYawAccelerationEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationLimit::YAWACCELERATION_RPS2));
}

void ReportAccelerationLimit::enableYawAcceleration(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationLimit::YAWACCELERATION_RPS2;
}

void ReportAccelerationLimit::disableYawAcceleration(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationLimit::YAWACCELERATION_RPS2);
}

} // namespace mobility
} // namespace openjaus


