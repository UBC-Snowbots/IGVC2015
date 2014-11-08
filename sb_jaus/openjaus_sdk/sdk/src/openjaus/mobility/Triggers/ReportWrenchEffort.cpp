
/**
\file ReportWrenchEffort.h

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
#include "openjaus/mobility/Triggers/ReportWrenchEffort.h"

namespace openjaus
{
namespace mobility
{

ReportWrenchEffort::ReportWrenchEffort() : 
	model::Message(),
	propulsiveLinearEffortX_percent(),
	propulsiveLinearEffortY_percent(),
	propulsiveLinearEffortZ_percent(),
	propulsiveRotationalEffortX_percent(),
	propulsiveRotationalEffortY_percent(),
	propulsiveRotationalEffortZ_percent(),
	resistiveLinearEffortX_percent(),
	resistiveLinearEffortY_percent(),
	resistiveLinearEffortZ_percent(),
	resistiveRotationalEffortX_percent(),
	resistiveRotationalEffortY_percent(),
	resistiveRotationalEffortZ_percent()
{
	this->id = ReportWrenchEffort::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&propulsiveLinearEffortX_percent);
	propulsiveLinearEffortX_percent.setName("PropulsiveLinearEffortX");
	propulsiveLinearEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveLinearEffortY_percent);
	propulsiveLinearEffortY_percent.setName("PropulsiveLinearEffortY");
	propulsiveLinearEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveLinearEffortZ_percent);
	propulsiveLinearEffortZ_percent.setName("PropulsiveLinearEffortZ");
	propulsiveLinearEffortZ_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveRotationalEffortX_percent);
	propulsiveRotationalEffortX_percent.setName("PropulsiveRotationalEffortX");
	propulsiveRotationalEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveRotationalEffortY_percent);
	propulsiveRotationalEffortY_percent.setName("PropulsiveRotationalEffortY");
	propulsiveRotationalEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveRotationalEffortZ_percent);
	propulsiveRotationalEffortZ_percent.setName("PropulsiveRotationalEffortZ");
	propulsiveRotationalEffortZ_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveLinearEffortX_percent);
	resistiveLinearEffortX_percent.setName("ResistiveLinearEffortX");
	resistiveLinearEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveLinearEffortY_percent);
	resistiveLinearEffortY_percent.setName("ResistiveLinearEffortY");
	resistiveLinearEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveLinearEffortZ_percent);
	resistiveLinearEffortZ_percent.setName("ResistiveLinearEffortZ");
	resistiveLinearEffortZ_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveRotationalEffortX_percent);
	resistiveRotationalEffortX_percent.setName("ResistiveRotationalEffortX");
	resistiveRotationalEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveRotationalEffortY_percent);
	resistiveRotationalEffortY_percent.setName("ResistiveRotationalEffortY");
	resistiveRotationalEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveRotationalEffortZ_percent);
	resistiveRotationalEffortZ_percent.setName("ResistiveRotationalEffortZ");
	resistiveRotationalEffortZ_percent.setOptional(true);
	// Nothing to init

}

ReportWrenchEffort::ReportWrenchEffort(model::Message *message) :
	model::Message(message),
	propulsiveLinearEffortX_percent(),
	propulsiveLinearEffortY_percent(),
	propulsiveLinearEffortZ_percent(),
	propulsiveRotationalEffortX_percent(),
	propulsiveRotationalEffortY_percent(),
	propulsiveRotationalEffortZ_percent(),
	resistiveLinearEffortX_percent(),
	resistiveLinearEffortY_percent(),
	resistiveLinearEffortZ_percent(),
	resistiveRotationalEffortX_percent(),
	resistiveRotationalEffortY_percent(),
	resistiveRotationalEffortZ_percent()
{
	this->id = ReportWrenchEffort::ID; // Initialize id member
	setType(transport::JAUS_MESSAGE);

	this->presenceVector = 0;

	fields.push_back(&propulsiveLinearEffortX_percent);
	propulsiveLinearEffortX_percent.setName("PropulsiveLinearEffortX");
	propulsiveLinearEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveLinearEffortY_percent);
	propulsiveLinearEffortY_percent.setName("PropulsiveLinearEffortY");
	propulsiveLinearEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveLinearEffortZ_percent);
	propulsiveLinearEffortZ_percent.setName("PropulsiveLinearEffortZ");
	propulsiveLinearEffortZ_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveRotationalEffortX_percent);
	propulsiveRotationalEffortX_percent.setName("PropulsiveRotationalEffortX");
	propulsiveRotationalEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveRotationalEffortY_percent);
	propulsiveRotationalEffortY_percent.setName("PropulsiveRotationalEffortY");
	propulsiveRotationalEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&propulsiveRotationalEffortZ_percent);
	propulsiveRotationalEffortZ_percent.setName("PropulsiveRotationalEffortZ");
	propulsiveRotationalEffortZ_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveLinearEffortX_percent);
	resistiveLinearEffortX_percent.setName("ResistiveLinearEffortX");
	resistiveLinearEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveLinearEffortY_percent);
	resistiveLinearEffortY_percent.setName("ResistiveLinearEffortY");
	resistiveLinearEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveLinearEffortZ_percent);
	resistiveLinearEffortZ_percent.setName("ResistiveLinearEffortZ");
	resistiveLinearEffortZ_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveRotationalEffortX_percent);
	resistiveRotationalEffortX_percent.setName("ResistiveRotationalEffortX");
	resistiveRotationalEffortX_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveRotationalEffortY_percent);
	resistiveRotationalEffortY_percent.setName("ResistiveRotationalEffortY");
	resistiveRotationalEffortY_percent.setOptional(true);
	// Nothing to init

	fields.push_back(&resistiveRotationalEffortZ_percent);
	resistiveRotationalEffortZ_percent.setName("ResistiveRotationalEffortZ");
	resistiveRotationalEffortZ_percent.setOptional(true);
	// Nothing to init


	system::Buffer *payloadBuffer = dynamic_cast<system::Buffer *>(message->getPayload());
	if(payloadBuffer)
	{
		this->from(payloadBuffer);
		payloadBuffer->reset();
	}
}

ReportWrenchEffort::~ReportWrenchEffort()
{

}


double ReportWrenchEffort::getPropulsiveLinearEffortX_percent(void)
{
	return this->propulsiveLinearEffortX_percent.getValue();
}

void ReportWrenchEffort::setPropulsiveLinearEffortX_percent(double value)
{
	this->propulsiveLinearEffortX_percent.setValue(value);
}

double ReportWrenchEffort::getPropulsiveLinearEffortY_percent(void)
{
	return this->propulsiveLinearEffortY_percent.getValue();
}

void ReportWrenchEffort::setPropulsiveLinearEffortY_percent(double value)
{
	this->propulsiveLinearEffortY_percent.setValue(value);
}

double ReportWrenchEffort::getPropulsiveLinearEffortZ_percent(void)
{
	return this->propulsiveLinearEffortZ_percent.getValue();
}

void ReportWrenchEffort::setPropulsiveLinearEffortZ_percent(double value)
{
	this->propulsiveLinearEffortZ_percent.setValue(value);
}

double ReportWrenchEffort::getPropulsiveRotationalEffortX_percent(void)
{
	return this->propulsiveRotationalEffortX_percent.getValue();
}

void ReportWrenchEffort::setPropulsiveRotationalEffortX_percent(double value)
{
	this->propulsiveRotationalEffortX_percent.setValue(value);
}

double ReportWrenchEffort::getPropulsiveRotationalEffortY_percent(void)
{
	return this->propulsiveRotationalEffortY_percent.getValue();
}

void ReportWrenchEffort::setPropulsiveRotationalEffortY_percent(double value)
{
	this->propulsiveRotationalEffortY_percent.setValue(value);
}

double ReportWrenchEffort::getPropulsiveRotationalEffortZ_percent(void)
{
	return this->propulsiveRotationalEffortZ_percent.getValue();
}

void ReportWrenchEffort::setPropulsiveRotationalEffortZ_percent(double value)
{
	this->propulsiveRotationalEffortZ_percent.setValue(value);
}

double ReportWrenchEffort::getResistiveLinearEffortX_percent(void)
{
	return this->resistiveLinearEffortX_percent.getValue();
}

void ReportWrenchEffort::setResistiveLinearEffortX_percent(double value)
{
	this->resistiveLinearEffortX_percent.setValue(value);
}

double ReportWrenchEffort::getResistiveLinearEffortY_percent(void)
{
	return this->resistiveLinearEffortY_percent.getValue();
}

void ReportWrenchEffort::setResistiveLinearEffortY_percent(double value)
{
	this->resistiveLinearEffortY_percent.setValue(value);
}

double ReportWrenchEffort::getResistiveLinearEffortZ_percent(void)
{
	return this->resistiveLinearEffortZ_percent.getValue();
}

void ReportWrenchEffort::setResistiveLinearEffortZ_percent(double value)
{
	this->resistiveLinearEffortZ_percent.setValue(value);
}

double ReportWrenchEffort::getResistiveRotationalEffortX_percent(void)
{
	return this->resistiveRotationalEffortX_percent.getValue();
}

void ReportWrenchEffort::setResistiveRotationalEffortX_percent(double value)
{
	this->resistiveRotationalEffortX_percent.setValue(value);
}

double ReportWrenchEffort::getResistiveRotationalEffortY_percent(void)
{
	return this->resistiveRotationalEffortY_percent.getValue();
}

void ReportWrenchEffort::setResistiveRotationalEffortY_percent(double value)
{
	this->resistiveRotationalEffortY_percent.setValue(value);
}

double ReportWrenchEffort::getResistiveRotationalEffortZ_percent(void)
{
	return this->resistiveRotationalEffortZ_percent.getValue();
}

void ReportWrenchEffort::setResistiveRotationalEffortZ_percent(double value)
{
	this->resistiveRotationalEffortZ_percent.setValue(value);
}

int ReportWrenchEffort::to(system::Buffer *dst)
{
	int byteSize = dst->pack(this->id);
	byteSize += dst->pack(this->presenceVector);
	if(this->isPropulsiveLinearEffortXEnabled())
	{
		byteSize += dst->pack(propulsiveLinearEffortX_percent);
	}
	if(this->isPropulsiveLinearEffortYEnabled())
	{
		byteSize += dst->pack(propulsiveLinearEffortY_percent);
	}
	if(this->isPropulsiveLinearEffortZEnabled())
	{
		byteSize += dst->pack(propulsiveLinearEffortZ_percent);
	}
	if(this->isPropulsiveRotationalEffortXEnabled())
	{
		byteSize += dst->pack(propulsiveRotationalEffortX_percent);
	}
	if(this->isPropulsiveRotationalEffortYEnabled())
	{
		byteSize += dst->pack(propulsiveRotationalEffortY_percent);
	}
	if(this->isPropulsiveRotationalEffortZEnabled())
	{
		byteSize += dst->pack(propulsiveRotationalEffortZ_percent);
	}
	if(this->isResistiveLinearEffortXEnabled())
	{
		byteSize += dst->pack(resistiveLinearEffortX_percent);
	}
	if(this->isResistiveLinearEffortYEnabled())
	{
		byteSize += dst->pack(resistiveLinearEffortY_percent);
	}
	if(this->isResistiveLinearEffortZEnabled())
	{
		byteSize += dst->pack(resistiveLinearEffortZ_percent);
	}
	if(this->isResistiveRotationalEffortXEnabled())
	{
		byteSize += dst->pack(resistiveRotationalEffortX_percent);
	}
	if(this->isResistiveRotationalEffortYEnabled())
	{
		byteSize += dst->pack(resistiveRotationalEffortY_percent);
	}
	if(this->isResistiveRotationalEffortZEnabled())
	{
		byteSize += dst->pack(resistiveRotationalEffortZ_percent);
	}
	return byteSize;
}

int ReportWrenchEffort::from(system::Buffer *src)
{
	int byteSize = src->unpack(this->id);
	byteSize += src->unpack(this->presenceVector);
	if(this->isPropulsiveLinearEffortXEnabled())
	{
		byteSize += src->unpack(propulsiveLinearEffortX_percent);
	}
	if(this->isPropulsiveLinearEffortYEnabled())
	{
		byteSize += src->unpack(propulsiveLinearEffortY_percent);
	}
	if(this->isPropulsiveLinearEffortZEnabled())
	{
		byteSize += src->unpack(propulsiveLinearEffortZ_percent);
	}
	if(this->isPropulsiveRotationalEffortXEnabled())
	{
		byteSize += src->unpack(propulsiveRotationalEffortX_percent);
	}
	if(this->isPropulsiveRotationalEffortYEnabled())
	{
		byteSize += src->unpack(propulsiveRotationalEffortY_percent);
	}
	if(this->isPropulsiveRotationalEffortZEnabled())
	{
		byteSize += src->unpack(propulsiveRotationalEffortZ_percent);
	}
	if(this->isResistiveLinearEffortXEnabled())
	{
		byteSize += src->unpack(resistiveLinearEffortX_percent);
	}
	if(this->isResistiveLinearEffortYEnabled())
	{
		byteSize += src->unpack(resistiveLinearEffortY_percent);
	}
	if(this->isResistiveLinearEffortZEnabled())
	{
		byteSize += src->unpack(resistiveLinearEffortZ_percent);
	}
	if(this->isResistiveRotationalEffortXEnabled())
	{
		byteSize += src->unpack(resistiveRotationalEffortX_percent);
	}
	if(this->isResistiveRotationalEffortYEnabled())
	{
		byteSize += src->unpack(resistiveRotationalEffortY_percent);
	}
	if(this->isResistiveRotationalEffortZEnabled())
	{
		byteSize += src->unpack(resistiveRotationalEffortZ_percent);
	}
	return byteSize;
}

int ReportWrenchEffort::length(void)
{
	int length = 0;
	length += sizeof(uint16_t); // Message ID
	length += sizeof(uint16_t); // PresenceVector
	if(this->isPropulsiveLinearEffortXEnabled())
	{
		length += propulsiveLinearEffortX_percent.length(); // propulsiveLinearEffortX_percent
	}
	if(this->isPropulsiveLinearEffortYEnabled())
	{
		length += propulsiveLinearEffortY_percent.length(); // propulsiveLinearEffortY_percent
	}
	if(this->isPropulsiveLinearEffortZEnabled())
	{
		length += propulsiveLinearEffortZ_percent.length(); // propulsiveLinearEffortZ_percent
	}
	if(this->isPropulsiveRotationalEffortXEnabled())
	{
		length += propulsiveRotationalEffortX_percent.length(); // propulsiveRotationalEffortX_percent
	}
	if(this->isPropulsiveRotationalEffortYEnabled())
	{
		length += propulsiveRotationalEffortY_percent.length(); // propulsiveRotationalEffortY_percent
	}
	if(this->isPropulsiveRotationalEffortZEnabled())
	{
		length += propulsiveRotationalEffortZ_percent.length(); // propulsiveRotationalEffortZ_percent
	}
	if(this->isResistiveLinearEffortXEnabled())
	{
		length += resistiveLinearEffortX_percent.length(); // resistiveLinearEffortX_percent
	}
	if(this->isResistiveLinearEffortYEnabled())
	{
		length += resistiveLinearEffortY_percent.length(); // resistiveLinearEffortY_percent
	}
	if(this->isResistiveLinearEffortZEnabled())
	{
		length += resistiveLinearEffortZ_percent.length(); // resistiveLinearEffortZ_percent
	}
	if(this->isResistiveRotationalEffortXEnabled())
	{
		length += resistiveRotationalEffortX_percent.length(); // resistiveRotationalEffortX_percent
	}
	if(this->isResistiveRotationalEffortYEnabled())
	{
		length += resistiveRotationalEffortY_percent.length(); // resistiveRotationalEffortY_percent
	}
	if(this->isResistiveRotationalEffortZEnabled())
	{
		length += resistiveRotationalEffortZ_percent.length(); // resistiveRotationalEffortZ_percent
	}
	return length;
}

std::string ReportWrenchEffort::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Message name=\"ReportWrenchEffort\"";
	oss << " id=\"0x4405\" >\n";
	oss << prefix.str() << "\t" << "<PresenceVector size=\"uint16_t\">\n";
	oss << prefix.str() << "\t" << "\t" << "<value>0x" << std::hex << (uint64_t)this->getPresenceVector() << std::dec << "</value>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPropulsiveLinearEffortXEnabled value=\"" << std::boolalpha << this->isPropulsiveLinearEffortXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPropulsiveLinearEffortYEnabled value=\"" << std::boolalpha << this->isPropulsiveLinearEffortYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPropulsiveLinearEffortZEnabled value=\"" << std::boolalpha << this->isPropulsiveLinearEffortZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPropulsiveRotationalEffortXEnabled value=\"" << std::boolalpha << this->isPropulsiveRotationalEffortXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPropulsiveRotationalEffortYEnabled value=\"" << std::boolalpha << this->isPropulsiveRotationalEffortYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isPropulsiveRotationalEffortZEnabled value=\"" << std::boolalpha << this->isPropulsiveRotationalEffortZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isResistiveLinearEffortXEnabled value=\"" << std::boolalpha << this->isResistiveLinearEffortXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isResistiveLinearEffortYEnabled value=\"" << std::boolalpha << this->isResistiveLinearEffortYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isResistiveLinearEffortZEnabled value=\"" << std::boolalpha << this->isResistiveLinearEffortZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isResistiveRotationalEffortXEnabled value=\"" << std::boolalpha << this->isResistiveRotationalEffortXEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isResistiveRotationalEffortYEnabled value=\"" << std::boolalpha << this->isResistiveRotationalEffortYEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "\t" << "<isResistiveRotationalEffortZEnabled value=\"" << std::boolalpha << this->isResistiveRotationalEffortZEnabled() << "\"/>\n";
	oss << prefix.str() << "\t" << "</PresenceVector>\n";
	if(this->isPropulsiveLinearEffortXEnabled())
	{
		oss << propulsiveLinearEffortX_percent.toXml(level+1); // propulsiveLinearEffortX_percent
	}
	if(this->isPropulsiveLinearEffortYEnabled())
	{
		oss << propulsiveLinearEffortY_percent.toXml(level+1); // propulsiveLinearEffortY_percent
	}
	if(this->isPropulsiveLinearEffortZEnabled())
	{
		oss << propulsiveLinearEffortZ_percent.toXml(level+1); // propulsiveLinearEffortZ_percent
	}
	if(this->isPropulsiveRotationalEffortXEnabled())
	{
		oss << propulsiveRotationalEffortX_percent.toXml(level+1); // propulsiveRotationalEffortX_percent
	}
	if(this->isPropulsiveRotationalEffortYEnabled())
	{
		oss << propulsiveRotationalEffortY_percent.toXml(level+1); // propulsiveRotationalEffortY_percent
	}
	if(this->isPropulsiveRotationalEffortZEnabled())
	{
		oss << propulsiveRotationalEffortZ_percent.toXml(level+1); // propulsiveRotationalEffortZ_percent
	}
	if(this->isResistiveLinearEffortXEnabled())
	{
		oss << resistiveLinearEffortX_percent.toXml(level+1); // resistiveLinearEffortX_percent
	}
	if(this->isResistiveLinearEffortYEnabled())
	{
		oss << resistiveLinearEffortY_percent.toXml(level+1); // resistiveLinearEffortY_percent
	}
	if(this->isResistiveLinearEffortZEnabled())
	{
		oss << resistiveLinearEffortZ_percent.toXml(level+1); // resistiveLinearEffortZ_percent
	}
	if(this->isResistiveRotationalEffortXEnabled())
	{
		oss << resistiveRotationalEffortX_percent.toXml(level+1); // resistiveRotationalEffortX_percent
	}
	if(this->isResistiveRotationalEffortYEnabled())
	{
		oss << resistiveRotationalEffortY_percent.toXml(level+1); // resistiveRotationalEffortY_percent
	}
	if(this->isResistiveRotationalEffortZEnabled())
	{
		oss << resistiveRotationalEffortZ_percent.toXml(level+1); // resistiveRotationalEffortZ_percent
	}
	oss << prefix.str() << "</Message>\n";
	return oss.str();
}

void ReportWrenchEffort::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t ReportWrenchEffort::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool ReportWrenchEffort::isPropulsiveLinearEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTX_PERCENT));
}

void ReportWrenchEffort::enablePropulsiveLinearEffortX(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTX_PERCENT;
}

void ReportWrenchEffort::disablePropulsiveLinearEffortX(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTX_PERCENT);
}

bool ReportWrenchEffort::isPropulsiveLinearEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTY_PERCENT));
}

void ReportWrenchEffort::enablePropulsiveLinearEffortY(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTY_PERCENT;
}

void ReportWrenchEffort::disablePropulsiveLinearEffortY(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTY_PERCENT);
}

bool ReportWrenchEffort::isPropulsiveLinearEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTZ_PERCENT));
}

void ReportWrenchEffort::enablePropulsiveLinearEffortZ(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTZ_PERCENT;
}

void ReportWrenchEffort::disablePropulsiveLinearEffortZ(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::PROPULSIVELINEAREFFORTZ_PERCENT);
}

bool ReportWrenchEffort::isPropulsiveRotationalEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTX_PERCENT));
}

void ReportWrenchEffort::enablePropulsiveRotationalEffortX(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTX_PERCENT;
}

void ReportWrenchEffort::disablePropulsiveRotationalEffortX(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTX_PERCENT);
}

bool ReportWrenchEffort::isPropulsiveRotationalEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTY_PERCENT));
}

void ReportWrenchEffort::enablePropulsiveRotationalEffortY(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTY_PERCENT;
}

void ReportWrenchEffort::disablePropulsiveRotationalEffortY(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTY_PERCENT);
}

bool ReportWrenchEffort::isPropulsiveRotationalEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTZ_PERCENT));
}

void ReportWrenchEffort::enablePropulsiveRotationalEffortZ(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTZ_PERCENT;
}

void ReportWrenchEffort::disablePropulsiveRotationalEffortZ(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::PROPULSIVEROTATIONALEFFORTZ_PERCENT);
}

bool ReportWrenchEffort::isResistiveLinearEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTX_PERCENT));
}

void ReportWrenchEffort::enableResistiveLinearEffortX(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTX_PERCENT;
}

void ReportWrenchEffort::disableResistiveLinearEffortX(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTX_PERCENT);
}

bool ReportWrenchEffort::isResistiveLinearEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTY_PERCENT));
}

void ReportWrenchEffort::enableResistiveLinearEffortY(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTY_PERCENT;
}

void ReportWrenchEffort::disableResistiveLinearEffortY(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTY_PERCENT);
}

bool ReportWrenchEffort::isResistiveLinearEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTZ_PERCENT));
}

void ReportWrenchEffort::enableResistiveLinearEffortZ(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTZ_PERCENT;
}

void ReportWrenchEffort::disableResistiveLinearEffortZ(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::RESISTIVELINEAREFFORTZ_PERCENT);
}

bool ReportWrenchEffort::isResistiveRotationalEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTX_PERCENT));
}

void ReportWrenchEffort::enableResistiveRotationalEffortX(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTX_PERCENT;
}

void ReportWrenchEffort::disableResistiveRotationalEffortX(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTX_PERCENT);
}

bool ReportWrenchEffort::isResistiveRotationalEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTY_PERCENT));
}

void ReportWrenchEffort::enableResistiveRotationalEffortY(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTY_PERCENT;
}

void ReportWrenchEffort::disableResistiveRotationalEffortY(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTY_PERCENT);
}

bool ReportWrenchEffort::isResistiveRotationalEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTZ_PERCENT));
}

void ReportWrenchEffort::enableResistiveRotationalEffortZ(void)
{
	this->presenceVector |= 0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTZ_PERCENT;
}

void ReportWrenchEffort::disableResistiveRotationalEffortZ(void)
{
	this->presenceVector &= ~(0x01 << ReportWrenchEffort::RESISTIVEROTATIONALEFFORTZ_PERCENT);
}

} // namespace mobility
} // namespace openjaus


