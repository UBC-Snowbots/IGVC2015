/**
\file VelocityCommandRecord.h

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
#include "openjaus/mobility/Triggers/Fields/VelocityCommandRecord.h"

namespace openjaus
{
namespace mobility
{

VelocityCommandRecord::VelocityCommandRecord():
	commandType(),
	velocityX_mps(),
	velocityY_mps(),
	velocityZ_mps(),
	rollRate_rps(),
	pitchRate_rps(),
	yawRate_rps()
{
	this->presenceVector = 0;

	fields.push_back(&commandType);
	commandType.setName("CommandType");
	commandType.setOptional(false);
	// Nothing to init

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

}

VelocityCommandRecord::VelocityCommandRecord(const VelocityCommandRecord &source)
{
	this->copy(const_cast<VelocityCommandRecord&>(source));
}

VelocityCommandRecord::~VelocityCommandRecord()
{

}


VelocityCommandTypeEnumeration::VelocityCommandTypeEnum VelocityCommandRecord::getCommandType(void)
{
	return this->commandType.getValue();
}

void VelocityCommandRecord::setCommandType(VelocityCommandTypeEnumeration::VelocityCommandTypeEnum value)
{
	this->commandType.setValue(value);
}

double VelocityCommandRecord::getVelocityX_mps(void)
{
	return this->velocityX_mps.getValue();
}

void VelocityCommandRecord::setVelocityX_mps(double value)
{
	this->velocityX_mps.setValue(value);
}

double VelocityCommandRecord::getVelocityY_mps(void)
{
	return this->velocityY_mps.getValue();
}

void VelocityCommandRecord::setVelocityY_mps(double value)
{
	this->velocityY_mps.setValue(value);
}

double VelocityCommandRecord::getVelocityZ_mps(void)
{
	return this->velocityZ_mps.getValue();
}

void VelocityCommandRecord::setVelocityZ_mps(double value)
{
	this->velocityZ_mps.setValue(value);
}

double VelocityCommandRecord::getRollRate_rps(void)
{
	return this->rollRate_rps.getValue();
}

void VelocityCommandRecord::setRollRate_rps(double value)
{
	this->rollRate_rps.setValue(value);
}

double VelocityCommandRecord::getPitchRate_rps(void)
{
	return this->pitchRate_rps.getValue();
}

void VelocityCommandRecord::setPitchRate_rps(double value)
{
	this->pitchRate_rps.setValue(value);
}

double VelocityCommandRecord::getYawRate_rps(void)
{
	return this->yawRate_rps.getValue();
}

void VelocityCommandRecord::setYawRate_rps(double value)
{
	this->yawRate_rps.setValue(value);
}

int VelocityCommandRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
	byteSize += dst->pack(this->presenceVector);
	byteSize += dst->pack(commandType);
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
	return byteSize;
}
int VelocityCommandRecord::from(system::Buffer *src)
{
	int byteSize = 0;
	byteSize += src->unpack(this->presenceVector);
	byteSize += src->unpack(commandType);
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
	return byteSize;
}

int VelocityCommandRecord::length(void)
{
	int length = 0;
	length += sizeof(uint8_t); // PresenceVector
	length += commandType.length(); // commandType
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
	return length;
}

std::string VelocityCommandRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"VelocityCommandRecord\">\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint8_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityXEnabled value=\"" << std::boolalpha << this->isVelocityXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityYEnabled value=\"" << std::boolalpha << this->isVelocityYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isVelocityZEnabled value=\"" << std::boolalpha << this->isVelocityZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isRollRateEnabled value=\"" << std::boolalpha << this->isRollRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPitchRateEnabled value=\"" << std::boolalpha << this->isPitchRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isYawRateEnabled value=\"" << std::boolalpha << this->isYawRateEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	oss << commandType.toXml(level+1); // commandType
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
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void VelocityCommandRecord::setPresenceVector(uint8_t value)
{
	this->presenceVector = value;
}

uint8_t VelocityCommandRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool VelocityCommandRecord::isVelocityXEnabled(void) const
{
	return (this->presenceVector & (0x01 << VelocityCommandRecord::VELOCITYX_MPS));
}

void VelocityCommandRecord::enableVelocityX(void)
{
	this->presenceVector |= 0x01 << VelocityCommandRecord::VELOCITYX_MPS;
}

void VelocityCommandRecord::disableVelocityX(void)
{
	this->presenceVector &= ~(0x01 << VelocityCommandRecord::VELOCITYX_MPS);
}

bool VelocityCommandRecord::isVelocityYEnabled(void) const
{
	return (this->presenceVector & (0x01 << VelocityCommandRecord::VELOCITYY_MPS));
}

void VelocityCommandRecord::enableVelocityY(void)
{
	this->presenceVector |= 0x01 << VelocityCommandRecord::VELOCITYY_MPS;
}

void VelocityCommandRecord::disableVelocityY(void)
{
	this->presenceVector &= ~(0x01 << VelocityCommandRecord::VELOCITYY_MPS);
}

bool VelocityCommandRecord::isVelocityZEnabled(void) const
{
	return (this->presenceVector & (0x01 << VelocityCommandRecord::VELOCITYZ_MPS));
}

void VelocityCommandRecord::enableVelocityZ(void)
{
	this->presenceVector |= 0x01 << VelocityCommandRecord::VELOCITYZ_MPS;
}

void VelocityCommandRecord::disableVelocityZ(void)
{
	this->presenceVector &= ~(0x01 << VelocityCommandRecord::VELOCITYZ_MPS);
}

bool VelocityCommandRecord::isRollRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << VelocityCommandRecord::ROLLRATE_RPS));
}

void VelocityCommandRecord::enableRollRate(void)
{
	this->presenceVector |= 0x01 << VelocityCommandRecord::ROLLRATE_RPS;
}

void VelocityCommandRecord::disableRollRate(void)
{
	this->presenceVector &= ~(0x01 << VelocityCommandRecord::ROLLRATE_RPS);
}

bool VelocityCommandRecord::isPitchRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << VelocityCommandRecord::PITCHRATE_RPS));
}

void VelocityCommandRecord::enablePitchRate(void)
{
	this->presenceVector |= 0x01 << VelocityCommandRecord::PITCHRATE_RPS;
}

void VelocityCommandRecord::disablePitchRate(void)
{
	this->presenceVector &= ~(0x01 << VelocityCommandRecord::PITCHRATE_RPS);
}

bool VelocityCommandRecord::isYawRateEnabled(void) const
{
	return (this->presenceVector & (0x01 << VelocityCommandRecord::YAWRATE_RPS));
}

void VelocityCommandRecord::enableYawRate(void)
{
	this->presenceVector |= 0x01 << VelocityCommandRecord::YAWRATE_RPS;
}

void VelocityCommandRecord::disableYawRate(void)
{
	this->presenceVector &= ~(0x01 << VelocityCommandRecord::YAWRATE_RPS);
}


void VelocityCommandRecord::copy(VelocityCommandRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->commandType.setName("VelocityCommandType");
	this->commandType.setOptional(false);
	this->commandType.setValue(source.getCommandType()); 
 
	this->velocityX_mps.setName("LinearVelocity");
	this->velocityX_mps.setOptional(false);
	this->velocityX_mps.setValue(source.getVelocityX_mps()); 
 
	this->velocityY_mps.setName("LinearVelocity");
	this->velocityY_mps.setOptional(false);
	this->velocityY_mps.setValue(source.getVelocityY_mps()); 
 
	this->velocityZ_mps.setName("LinearVelocity");
	this->velocityZ_mps.setOptional(false);
	this->velocityZ_mps.setValue(source.getVelocityZ_mps()); 
 
	this->rollRate_rps.setName("AngularVelocity");
	this->rollRate_rps.setOptional(false);
	this->rollRate_rps.setValue(source.getRollRate_rps()); 
 
	this->pitchRate_rps.setName("AngularVelocity");
	this->pitchRate_rps.setOptional(false);
	this->pitchRate_rps.setValue(source.getPitchRate_rps()); 
 
	this->yawRate_rps.setName("AngularVelocity");
	this->yawRate_rps.setOptional(false);
	this->yawRate_rps.setValue(source.getYawRate_rps()); 
 
}

} // namespace mobility
} // namespace openjaus

