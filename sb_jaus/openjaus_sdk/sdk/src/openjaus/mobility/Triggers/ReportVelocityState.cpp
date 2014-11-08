
/**
\file ReportVelocityState.h

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
#include "openjaus/mobility/Triggers/ReportVelocityState.h"

namespace openjaus
{
namespace mobility
{

ReportVelocityState::ReportVelocityState() : 
	model::Message(),
	velocityX_mps(),
	velocityY_mps(),
	velocityZ_mps(),
	velocityRms_mps(),
	rollRate_rps(),
	pitchRate_rps(),
	yawRate_rps(),
	rateRms_rps(),
	timeStamp()
{
	this->id = ReportVelocityState::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&velocityX_mps);
	velocityX_mps.setName("VelocityX");
	velocityX_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&velocityY_mps);
	velocityY_mps.setName("VelocityY");
	velocityY_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&velocityZ_mps);
	velocityZ_mps.setName("VelocityZ");
	velocityZ_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&velocityRms_mps);
	velocityRms_mps.setName("VelocityRms");
	velocityRms_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&rollRate_rps);
	rollRate_rps.setName("RollRate");
	rollRate_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&pitchRate_rps);
	pitchRate_rps.setName("PitchRate");
	pitchRate_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&yawRate_rps);
	yawRate_rps.setName("YawRate");
	yawRate_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&rateRms_rps);
	rateRms_rps.setName("RateRms");
	rateRms_rps.setOptional(false);
	// Nothing to init

	fields.push_back(&timeStamp);
	timeStamp.setName("TimeStamp");
	timeStamp.setOptional(true);
	// Nothing

}

ReportVelocityState::ReportVelocityState(model::Message *message) :
	model::Message(message),
	velocityX_mps(),
	velocityY_mps(),
	velocityZ_mps(),
	velocityRms_mps(),
	rollRate_rps(),
	pitchRate_rps(),
	yawRate_rps(),
	rateRms_rps(),
	timeStamp()
{
	this->id = ReportVelocityState::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&velocityX_mps);
	velocityX_mps.setName("VelocityX");
	velocityX_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&velocityY_mps);
	velocityY_mps.setName("VelocityY");
	velocityY_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&velocityZ_mps);
	velocityZ_mps.setName("VelocityZ");
	velocityZ_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&velocityRms_mps);
	velocityRms_mps.setName("VelocityRms");
	velocityRms_mps.setOptional(true);
	// Nothing to init

	fields.push_back(&rollRate_rps);
	rollRate_rps.setName("RollRate");
	rollRate_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&pitchRate_rps);
	pitchRate_rps.setName("PitchRate");
	pitchRate_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&yawRate_rps);
	yawRate_rps.setName("YawRate");
	yawRate_rps.setOptional(true);
	// Nothing to init

	fields.push_back(&rateRms_rps);
	rateRms_rps.setName("RateRms");
	rateRms_rps.setOptional(false);
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

ReportVelocityState::~ReportVelocityState()
{

}


double ReportVelocityState::getVelocityX_mps(void)
{
	return this->velocityX_mps.getValue();
}

void ReportVelocityState::setVelocityX_mps(double value)
{
	this->velocityX_mps.setValue(value);
}

double ReportVelocityState::getVelocityY_mps(void)
{
	return this->velocityY_mps.getValue();
}

void ReportVelocityState::setVelocityY_mps(double value)
{
	this->velocityY_mps.setValue(value);
}

double ReportVelocityState::getVelocityZ_mps(void)
{
	return this->velocityZ_mps.getValue();
}

void ReportVelocityState::setVelocityZ_mps(double value)
{
	this->velocityZ_mps.setValue(value);
}

double ReportVelocityState::getVelocityRms_mps(void)
{
	return this->velocityRms_mps.getValue();
}

void ReportVelocityState::setVelocityRms_mps(double value)
{
	this->velocityRms_mps.setValue(value);
}

double ReportVelocityState::getRollRate_rps(void)
{
	return this->rollRate_rps.getValue();
}

void ReportVelocityState::setRollRate_rps(double value)
{
	this->rollRate_rps.setValue(value);
}

double ReportVelocityState::getPitchRate_rps(void)
{
	return this->pitchRate_rps.getValue();
}

void ReportVelocityState::setPitchRate_rps(double value)
{
	this->pitchRate_rps.setValue(value);
}

double ReportVelocityState::getYawRate_rps(void)
{
	return this->yawRate_rps.getValue();
}

void ReportVelocityState::setYawRate_rps(double value)
{
	this->yawRate_rps.setValue(value);
}

double ReportVelocityState::getRateRms_rps(void)
{
	return this->rateRms_rps.getValue();
}

void ReportVelocityState::setRateRms_rps(double value)
{
	this->rateRms_rps.setValue(value);
}

JausTimeStampBitField& ReportVelocityState::getTimeStamp(void)
{
	return this->timeStamp;
}

int ReportVelocityState::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isVelocityXEnabled())
	{
		byteSize += dst->pack(velocityX_mps);
	}
	if(this->isVelocityYEnabled())
	{
		byteSize += dst->pack(velocityY_mps);
	}
	if(this->isVelocityZEnabled())
	{
		byteSize += dst->pack(velocityZ_mps);
	}
	if(this->isVelocityRmsEnabled())
	{
		byteSize += dst->pack(velocityRms_mps);
	}
	if(this->isRollRateEnabled())
	{
		byteSize += dst->pack(rollRate_rps);
	}
	if(this->isPitchRateEnabled())
	{
		byteSize += dst->pack(pitchRate_rps);
	}
	if(this->isYawRateEnabled())
	{
		byteSize += dst->pack(yawRate_rps);
	}
	byteSize += dst->pack(rateRms_rps);
	if(this->isTimeStampEnabled())
	{
		byteSize += dst->pack(timeStamp);
	}
	return byteSize;
}

int ReportVelocityState::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isVelocityXEnabled())
	{
		byteSize += src->unpack(velocityX_mps);
	}
	if(this->isVelocityYEnabled())
	{
		byteSize += src->unpack(velocityY_mps);
	}
	if(this->isVelocityZEnabled())
	{
		byteSize += src->unpack(velocityZ_mps);
	}
	if(this->isVelocityRmsEnabled())
	{
		byteSize += src->unpack(velocityRms_mps);
	}
	if(this->isRollRateEnabled())
	{
		byteSize += src->unpack(rollRate_rps);
	}
	if(this->isPitchRateEnabled())
	{
		byteSize += src->unpack(pitchRate_rps);
	}
	if(this->isYawRateEnabled())
	{
		byteSize += src->unpack(yawRate_rps);
	}
	byteSize += src->unpack(rateRms_rps);
	if(this->isTimeStampEnabled())
	{
		byteSize += src->unpack(timeStamp);
	}
	return byteSize;
}

int ReportVelocityState::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint8_t); // PresenceVector
	if(this->isVelocityXEnabled())
	{
		length += velocityX_mps.length(); // velocityX_mps
	}
	if(this->isVelocityYEnabled())
	{
		length += velocityY_mps.length(); // velocityY_mps
	}
	if(this->isVelocityZEnabled())
	{
		length += velocityZ_mps.length(); // velocityZ_mps
	}
	if(this->isVelocityRmsEnabled())
	{
		length += velocityRms_mps.length(); // velocityRms_mps
	}
	if(this->isRollRateEnabled())
	{
		length += rollRate_rps.length(); // rollRate_rps
	}
	if(this->isPitchRateEnabled())
	{
		length += pitchRate_rps.length(); // pitchRate_rps
	}
	if(this->isYawRateEnabled())
	{
		length += yawRate_rps.length(); // yawRate_rps
	}
	length += rateRms_rps.length(); // rateRms_rps
	if(this->isTimeStampEnabled())
	{
		length += timeStamp.length(); // timeStamp
	}
	return length;
}

std::string ReportVelocityState::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportVelocityState\"";
	oss << " id=\"0x4404\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityXEnabled value=\"" << std::boolalpha << this->isVelocityXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityYEnabled value=\"" << std::boolalpha << this->isVelocityYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityZEnabled value=\"" << std::boolalpha << this->isVelocityZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityRmsEnabled value=\"" << std::boolalpha << this->isVelocityRmsEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollRateEnabled value=\"" << std::boolalpha << this->isRollRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchRateEnabled value=\"" << std::boolalpha << this->isPitchRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawRateEnabled value=\"" << std::boolalpha << this->isYawRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isTimeStampEnabled value=\"" << std::boolalpha << this->isTimeStampEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isVelocityXEnabled())
	{
		oss << velocityX_mps.toXml(level+1); // velocityX_mps
	}
	if(this->isVelocityYEnabled())
	{
		oss << velocityY_mps.toXml(level+1); // velocityY_mps
	}
	if(this->isVelocityZEnabled())
	{
		oss << velocityZ_mps.toXml(level+1); // velocityZ_mps
	}
	if(this->isVelocityRmsEnabled())
	{
		oss << velocityRms_mps.toXml(level+1); // velocityRms_mps
	}
	if(this->isRollRateEnabled())
	{
		oss << rollRate_rps.toXml(level+1); // rollRate_rps
	}
	if(this->isPitchRateEnabled())
	{
		oss << pitchRate_rps.toXml(level+1); // pitchRate_rps
	}
	if(this->isYawRateEnabled())
	{
		oss << yawRate_rps.toXml(level+1); // yawRate_rps
	}
	oss << rateRms_rps.toXml(level+1); // rateRms_rps
	if(this->isTimeStampEnabled())
	{
		oss << timeStamp.toXml(level+1); // timeStamp
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportVelocityState::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t ReportVelocityState::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportVelocityState::isVelocityXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::VELOCITYX_MPS));
}

void ReportVelocityState::enableVelocityX(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::VELOCITYX_MPS;
}

void ReportVelocityState::disableVelocityX(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::VELOCITYX_MPS);
}

bool ReportVelocityState::isVelocityYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::VELOCITYY_MPS));
}

void ReportVelocityState::enableVelocityY(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::VELOCITYY_MPS;
}

void ReportVelocityState::disableVelocityY(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::VELOCITYY_MPS);
}

bool ReportVelocityState::isVelocityZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::VELOCITYZ_MPS));
}

void ReportVelocityState::enableVelocityZ(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::VELOCITYZ_MPS;
}

void ReportVelocityState::disableVelocityZ(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::VELOCITYZ_MPS);
}

bool ReportVelocityState::isVelocityRmsEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::VELOCITYRMS_MPS));
}

void ReportVelocityState::enableVelocityRms(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::VELOCITYRMS_MPS;
}

void ReportVelocityState::disableVelocityRms(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::VELOCITYRMS_MPS);
}

bool ReportVelocityState::isRollRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::ROLLRATE_RPS));
}

void ReportVelocityState::enableRollRate(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::ROLLRATE_RPS;
}

void ReportVelocityState::disableRollRate(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::ROLLRATE_RPS);
}

bool ReportVelocityState::isPitchRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::PITCHRATE_RPS));
}

void ReportVelocityState::enablePitchRate(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::PITCHRATE_RPS;
}

void ReportVelocityState::disablePitchRate(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::PITCHRATE_RPS);
}

bool ReportVelocityState::isYawRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::YAWRATE_RPS));
}

void ReportVelocityState::enableYawRate(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::YAWRATE_RPS;
}

void ReportVelocityState::disableYawRate(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::YAWRATE_RPS);
}

bool ReportVelocityState::isTimeStampEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportVelocityState::TIMESTAMP));
}

void ReportVelocityState::enableTimeStamp(void)
{
	this->presenceVector |= 0x01 << ReportVelocityState::TIMESTAMP;
}

void ReportVelocityState::disableTimeStamp(void)
{
	this->presenceVector &= ~(0x01 << ReportVelocityState::TIMESTAMP);
}

} // namespace mobility
} // namespace openjaus


