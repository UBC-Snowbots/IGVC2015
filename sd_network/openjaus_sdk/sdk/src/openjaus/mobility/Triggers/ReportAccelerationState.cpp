
/**
\file ReportAccelerationState.h

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
#include "openjaus/mobility/Triggers/ReportAccelerationState.h"

namespace openjaus
{
namespace mobility
{

ReportAccelerationState::ReportAccelerationState() : 
	model::Message(),
	timeStamp(),
	accelerationX_mps2(),
	accelerationY_mps2(),
	accelerationZ_mps2(),
	accelerationRms_mps2(),
	rollAcceleration_rps2(),
	pitchAcceleration_rps2(),
	yawAcceleration_rps2(),
	rotationalAccelerationRms_rps2()
{
	this->id = ReportAccelerationState::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(true);
	// Nothing

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

	fields.push_back(&accelerationRms_mps2);
	accelerationRms_mps2.setName("AccelerationRms");
	accelerationRms_mps2.setOptional(true);
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

	fields.push_back(&rotationalAccelerationRms_rps2);
	rotationalAccelerationRms_rps2.setName("RotationalAccelerationRms");
	rotationalAccelerationRms_rps2.setOptional(true);
	// Nothing to init

}

ReportAccelerationState::ReportAccelerationState(model::Message *message) :
	model::Message(message),
	timeStamp(),
	accelerationX_mps2(),
	accelerationY_mps2(),
	accelerationZ_mps2(),
	accelerationRms_mps2(),
	rollAcceleration_rps2(),
	pitchAcceleration_rps2(),
	yawAcceleration_rps2(),
	rotationalAccelerationRms_rps2()
{
	this->id = ReportAccelerationState::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(true);
	// Nothing

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

	fields.push_back(&accelerationRms_mps2);
	accelerationRms_mps2.setName("AccelerationRms");
	accelerationRms_mps2.setOptional(true);
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

	fields.push_back(&rotationalAccelerationRms_rps2);
	rotationalAccelerationRms_rps2.setName("RotationalAccelerationRms");
	rotationalAccelerationRms_rps2.setOptional(true);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportAccelerationState::~ReportAccelerationState()
{

}


JausTimeStampBitField& ReportAccelerationState::getTimeStamp(void)
{
	return this->timeStamp;
}

double ReportAccelerationState::getAccelerationX_mps2(void)
{
	return this->accelerationX_mps2.getValue();
}

void ReportAccelerationState::setAccelerationX_mps2(double value)
{
	this->accelerationX_mps2.setValue(value);
}

double ReportAccelerationState::getAccelerationY_mps2(void)
{
	return this->accelerationY_mps2.getValue();
}

void ReportAccelerationState::setAccelerationY_mps2(double value)
{
	this->accelerationY_mps2.setValue(value);
}

double ReportAccelerationState::getAccelerationZ_mps2(void)
{
	return this->accelerationZ_mps2.getValue();
}

void ReportAccelerationState::setAccelerationZ_mps2(double value)
{
	this->accelerationZ_mps2.setValue(value);
}

double ReportAccelerationState::getAccelerationRms_mps2(void)
{
	return this->accelerationRms_mps2.getValue();
}

void ReportAccelerationState::setAccelerationRms_mps2(double value)
{
	this->accelerationRms_mps2.setValue(value);
}

double ReportAccelerationState::getRollAcceleration_rps2(void)
{
	return this->rollAcceleration_rps2.getValue();
}

void ReportAccelerationState::setRollAcceleration_rps2(double value)
{
	this->rollAcceleration_rps2.setValue(value);
}

double ReportAccelerationState::getPitchAcceleration_rps2(void)
{
	return this->pitchAcceleration_rps2.getValue();
}

void ReportAccelerationState::setPitchAcceleration_rps2(double value)
{
	this->pitchAcceleration_rps2.setValue(value);
}

double ReportAccelerationState::getYawAcceleration_rps2(void)
{
	return this->yawAcceleration_rps2.getValue();
}

void ReportAccelerationState::setYawAcceleration_rps2(double value)
{
	this->yawAcceleration_rps2.setValue(value);
}

double ReportAccelerationState::getRotationalAccelerationRms_rps2(void)
{
	return this->rotationalAccelerationRms_rps2.getValue();
}

void ReportAccelerationState::setRotationalAccelerationRms_rps2(double value)
{
	this->rotationalAccelerationRms_rps2.setValue(value);
}

int ReportAccelerationState::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isTimeStampEnabled())
	{
		byteSize += dst->pack(timeStamp);
	}
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
	if(this->isAccelerationRmsEnabled())
	{
		byteSize += dst->pack(accelerationRms_mps2);
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
	if(this->isRotationalAccelerationRmsEnabled())
	{
		byteSize += dst->pack(rotationalAccelerationRms_rps2);
	}
	return byteSize;
}

int ReportAccelerationState::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isTimeStampEnabled())
	{
		byteSize += src->unpack(timeStamp);
	}
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
	if(this->isAccelerationRmsEnabled())
	{
		byteSize += src->unpack(accelerationRms_mps2);
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
	if(this->isRotationalAccelerationRmsEnabled())
	{
		byteSize += src->unpack(rotationalAccelerationRms_rps2);
	}
	return byteSize;
}

int ReportAccelerationState::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint16_t); // PresenceVector
	if(this->isTimeStampEnabled())
	{
		length += timeStamp.length(); // timeStamp
	}
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
	if(this->isAccelerationRmsEnabled())
	{
		length += accelerationRms_mps2.length(); // accelerationRms_mps2
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
	if(this->isRotationalAccelerationRmsEnabled())
	{
		length += rotationalAccelerationRms_rps2.length(); // rotationalAccelerationRms_rps2
	}
	return length;
}

std::string ReportAccelerationState::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportAccelerationState\"";
	oss << " id=\"0x4417\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTimeStampEnabled value=\"" << std::boolalpha << this->isTimeStampEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationXEnabled value=\"" << std::boolalpha << this->isAccelerationXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationYEnabled value=\"" << std::boolalpha << this->isAccelerationYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationZEnabled value=\"" << std::boolalpha << this->isAccelerationZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isAccelerationRmsEnabled value=\"" << std::boolalpha << this->isAccelerationRmsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollAccelerationEnabled value=\"" << std::boolalpha << this->isRollAccelerationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchAccelerationEnabled value=\"" << std::boolalpha << this->isPitchAccelerationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawAccelerationEnabled value=\"" << std::boolalpha << this->isYawAccelerationEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRotationalAccelerationRmsEnabled value=\"" << std::boolalpha << this->isRotationalAccelerationRmsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isTimeStampEnabled())
	{
		oss << timeStamp.toXml(level+1); // timeStamp
	}
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
	if(this->isAccelerationRmsEnabled())
	{
		oss << accelerationRms_mps2.toXml(level+1); // accelerationRms_mps2
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
	if(this->isRotationalAccelerationRmsEnabled())
	{
		oss << rotationalAccelerationRms_rps2.toXml(level+1); // rotationalAccelerationRms_rps2
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportAccelerationState::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t ReportAccelerationState::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportAccelerationState::isTimeStampEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::TIMESTAMP));
}

void ReportAccelerationState::enableTimeStamp(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::TIMESTAMP;
}

void ReportAccelerationState::disableTimeStamp(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::TIMESTAMP);
}

bool ReportAccelerationState::isAccelerationXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::ACCELERATIONX_MPS2));
}

void ReportAccelerationState::enableAccelerationX(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::ACCELERATIONX_MPS2;
}

void ReportAccelerationState::disableAccelerationX(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::ACCELERATIONX_MPS2);
}

bool ReportAccelerationState::isAccelerationYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::ACCELERATIONY_MPS2));
}

void ReportAccelerationState::enableAccelerationY(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::ACCELERATIONY_MPS2;
}

void ReportAccelerationState::disableAccelerationY(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::ACCELERATIONY_MPS2);
}

bool ReportAccelerationState::isAccelerationZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::ACCELERATIONZ_MPS2));
}

void ReportAccelerationState::enableAccelerationZ(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::ACCELERATIONZ_MPS2;
}

void ReportAccelerationState::disableAccelerationZ(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::ACCELERATIONZ_MPS2);
}

bool ReportAccelerationState::isAccelerationRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::ACCELERATIONRMS_MPS2));
}

void ReportAccelerationState::enableAccelerationRms(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::ACCELERATIONRMS_MPS2;
}

void ReportAccelerationState::disableAccelerationRms(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::ACCELERATIONRMS_MPS2);
}

bool ReportAccelerationState::isRollAccelerationEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::ROLLACCELERATION_RPS2));
}

void ReportAccelerationState::enableRollAcceleration(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::ROLLACCELERATION_RPS2;
}

void ReportAccelerationState::disableRollAcceleration(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::ROLLACCELERATION_RPS2);
}

bool ReportAccelerationState::isPitchAccelerationEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::PITCHACCELERATION_RPS2));
}

void ReportAccelerationState::enablePitchAcceleration(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::PITCHACCELERATION_RPS2;
}

void ReportAccelerationState::disablePitchAcceleration(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::PITCHACCELERATION_RPS2);
}

bool ReportAccelerationState::isYawAccelerationEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::YAWACCELERATION_RPS2));
}

void ReportAccelerationState::enableYawAcceleration(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::YAWACCELERATION_RPS2;
}

void ReportAccelerationState::disableYawAcceleration(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::YAWACCELERATION_RPS2);
}

bool ReportAccelerationState::isRotationalAccelerationRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportAccelerationState::ROTATIONALACCELERATIONRMS_RPS2));
}

void ReportAccelerationState::enableRotationalAccelerationRms(void)
{
	this->presenceVector |= 0x01 << ReportAccelerationState::ROTATIONALACCELERATIONRMS_RPS2;
}

void ReportAccelerationState::disableRotationalAccelerationRms(void)
{
	this->presenceVector &= ~(0x01 << ReportAccelerationState::ROTATIONALACCELERATIONRMS_RPS2);
}

} // namespace mobility
} // namespace openjaus


