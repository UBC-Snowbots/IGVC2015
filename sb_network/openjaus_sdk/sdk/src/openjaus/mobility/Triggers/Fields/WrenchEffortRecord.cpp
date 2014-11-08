/**
\file WrenchEffortRecord.h

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
#include "openjaus/mobility/Triggers/Fields/WrenchEffortRecord.h"

namespace openjaus
{
namespace mobility
{

WrenchEffortRecord::WrenchEffortRecord():
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

WrenchEffortRecord::WrenchEffortRecord(const WrenchEffortRecord &source)
{
	this->copy(const_cast<WrenchEffortRecord&>(source));
}

WrenchEffortRecord::~WrenchEffortRecord()
{

}


double WrenchEffortRecord::getPropulsiveLinearEffortX_percent(void)
{
	return this->propulsiveLinearEffortX_percent.getValue();
}

void WrenchEffortRecord::setPropulsiveLinearEffortX_percent(double value)
{
	this->propulsiveLinearEffortX_percent.setValue(value);
}

double WrenchEffortRecord::getPropulsiveLinearEffortY_percent(void)
{
	return this->propulsiveLinearEffortY_percent.getValue();
}

void WrenchEffortRecord::setPropulsiveLinearEffortY_percent(double value)
{
	this->propulsiveLinearEffortY_percent.setValue(value);
}

double WrenchEffortRecord::getPropulsiveLinearEffortZ_percent(void)
{
	return this->propulsiveLinearEffortZ_percent.getValue();
}

void WrenchEffortRecord::setPropulsiveLinearEffortZ_percent(double value)
{
	this->propulsiveLinearEffortZ_percent.setValue(value);
}

double WrenchEffortRecord::getPropulsiveRotationalEffortX_percent(void)
{
	return this->propulsiveRotationalEffortX_percent.getValue();
}

void WrenchEffortRecord::setPropulsiveRotationalEffortX_percent(double value)
{
	this->propulsiveRotationalEffortX_percent.setValue(value);
}

double WrenchEffortRecord::getPropulsiveRotationalEffortY_percent(void)
{
	return this->propulsiveRotationalEffortY_percent.getValue();
}

void WrenchEffortRecord::setPropulsiveRotationalEffortY_percent(double value)
{
	this->propulsiveRotationalEffortY_percent.setValue(value);
}

double WrenchEffortRecord::getPropulsiveRotationalEffortZ_percent(void)
{
	return this->propulsiveRotationalEffortZ_percent.getValue();
}

void WrenchEffortRecord::setPropulsiveRotationalEffortZ_percent(double value)
{
	this->propulsiveRotationalEffortZ_percent.setValue(value);
}

double WrenchEffortRecord::getResistiveLinearEffortX_percent(void)
{
	return this->resistiveLinearEffortX_percent.getValue();
}

void WrenchEffortRecord::setResistiveLinearEffortX_percent(double value)
{
	this->resistiveLinearEffortX_percent.setValue(value);
}

double WrenchEffortRecord::getResistiveLinearEffortY_percent(void)
{
	return this->resistiveLinearEffortY_percent.getValue();
}

void WrenchEffortRecord::setResistiveLinearEffortY_percent(double value)
{
	this->resistiveLinearEffortY_percent.setValue(value);
}

double WrenchEffortRecord::getResistiveLinearEffortZ_percent(void)
{
	return this->resistiveLinearEffortZ_percent.getValue();
}

void WrenchEffortRecord::setResistiveLinearEffortZ_percent(double value)
{
	this->resistiveLinearEffortZ_percent.setValue(value);
}

double WrenchEffortRecord::getResistiveRotationalEffortX_percent(void)
{
	return this->resistiveRotationalEffortX_percent.getValue();
}

void WrenchEffortRecord::setResistiveRotationalEffortX_percent(double value)
{
	this->resistiveRotationalEffortX_percent.setValue(value);
}

double WrenchEffortRecord::getResistiveRotationalEffortY_percent(void)
{
	return this->resistiveRotationalEffortY_percent.getValue();
}

void WrenchEffortRecord::setResistiveRotationalEffortY_percent(double value)
{
	this->resistiveRotationalEffortY_percent.setValue(value);
}

double WrenchEffortRecord::getResistiveRotationalEffortZ_percent(void)
{
	return this->resistiveRotationalEffortZ_percent.getValue();
}

void WrenchEffortRecord::setResistiveRotationalEffortZ_percent(double value)
{
	this->resistiveRotationalEffortZ_percent.setValue(value);
}

int WrenchEffortRecord::to(system::Buffer *dst)
{
	int byteSize = 0;
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
int WrenchEffortRecord::from(system::Buffer *src)
{
	int byteSize = 0;
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

int WrenchEffortRecord::length(void)
{
	int length = 0;
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

std::string WrenchEffortRecord::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<Record type=\"WrenchEffortRecord\">\n";
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
	oss << prefix.str() << "</Record>\n";
	return oss.str();
}


void WrenchEffortRecord::setPresenceVector(uint16_t value)
{
	this->presenceVector = value;
}

uint16_t WrenchEffortRecord::getPresenceVector(void) const
{
	return this->presenceVector;
}

bool WrenchEffortRecord::isPropulsiveLinearEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTX_PERCENT));
}

void WrenchEffortRecord::enablePropulsiveLinearEffortX(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTX_PERCENT;
}

void WrenchEffortRecord::disablePropulsiveLinearEffortX(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTX_PERCENT);
}

bool WrenchEffortRecord::isPropulsiveLinearEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTY_PERCENT));
}

void WrenchEffortRecord::enablePropulsiveLinearEffortY(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTY_PERCENT;
}

void WrenchEffortRecord::disablePropulsiveLinearEffortY(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTY_PERCENT);
}

bool WrenchEffortRecord::isPropulsiveLinearEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTZ_PERCENT));
}

void WrenchEffortRecord::enablePropulsiveLinearEffortZ(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTZ_PERCENT;
}

void WrenchEffortRecord::disablePropulsiveLinearEffortZ(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::PROPULSIVELINEAREFFORTZ_PERCENT);
}

bool WrenchEffortRecord::isPropulsiveRotationalEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTX_PERCENT));
}

void WrenchEffortRecord::enablePropulsiveRotationalEffortX(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTX_PERCENT;
}

void WrenchEffortRecord::disablePropulsiveRotationalEffortX(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTX_PERCENT);
}

bool WrenchEffortRecord::isPropulsiveRotationalEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTY_PERCENT));
}

void WrenchEffortRecord::enablePropulsiveRotationalEffortY(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTY_PERCENT;
}

void WrenchEffortRecord::disablePropulsiveRotationalEffortY(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTY_PERCENT);
}

bool WrenchEffortRecord::isPropulsiveRotationalEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTZ_PERCENT));
}

void WrenchEffortRecord::enablePropulsiveRotationalEffortZ(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTZ_PERCENT;
}

void WrenchEffortRecord::disablePropulsiveRotationalEffortZ(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::PROPULSIVEROTATIONALEFFORTZ_PERCENT);
}

bool WrenchEffortRecord::isResistiveLinearEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTX_PERCENT));
}

void WrenchEffortRecord::enableResistiveLinearEffortX(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTX_PERCENT;
}

void WrenchEffortRecord::disableResistiveLinearEffortX(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTX_PERCENT);
}

bool WrenchEffortRecord::isResistiveLinearEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTY_PERCENT));
}

void WrenchEffortRecord::enableResistiveLinearEffortY(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTY_PERCENT;
}

void WrenchEffortRecord::disableResistiveLinearEffortY(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTY_PERCENT);
}

bool WrenchEffortRecord::isResistiveLinearEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTZ_PERCENT));
}

void WrenchEffortRecord::enableResistiveLinearEffortZ(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTZ_PERCENT;
}

void WrenchEffortRecord::disableResistiveLinearEffortZ(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::RESISTIVELINEAREFFORTZ_PERCENT);
}

bool WrenchEffortRecord::isResistiveRotationalEffortXEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTX_PERCENT));
}

void WrenchEffortRecord::enableResistiveRotationalEffortX(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTX_PERCENT;
}

void WrenchEffortRecord::disableResistiveRotationalEffortX(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTX_PERCENT);
}

bool WrenchEffortRecord::isResistiveRotationalEffortYEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTY_PERCENT));
}

void WrenchEffortRecord::enableResistiveRotationalEffortY(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTY_PERCENT;
}

void WrenchEffortRecord::disableResistiveRotationalEffortY(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTY_PERCENT);
}

bool WrenchEffortRecord::isResistiveRotationalEffortZEnabled(void) const
{
	return (this->presenceVector & (0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTZ_PERCENT));
}

void WrenchEffortRecord::enableResistiveRotationalEffortZ(void)
{
	this->presenceVector |= 0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTZ_PERCENT;
}

void WrenchEffortRecord::disableResistiveRotationalEffortZ(void)
{
	this->presenceVector &= ~(0x01 << WrenchEffortRecord::RESISTIVEROTATIONALEFFORTZ_PERCENT);
}


void WrenchEffortRecord::copy(WrenchEffortRecord& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());
	
	this->presenceVector = source.getPresenceVector();
	this->propulsiveLinearEffortX_percent.setName("PropulsiveEffort");
	this->propulsiveLinearEffortX_percent.setOptional(false);
	this->propulsiveLinearEffortX_percent.setValue(source.getPropulsiveLinearEffortX_percent()); 
 
	this->propulsiveLinearEffortY_percent.setName("PropulsiveEffort");
	this->propulsiveLinearEffortY_percent.setOptional(false);
	this->propulsiveLinearEffortY_percent.setValue(source.getPropulsiveLinearEffortY_percent()); 
 
	this->propulsiveLinearEffortZ_percent.setName("PropulsiveEffort");
	this->propulsiveLinearEffortZ_percent.setOptional(false);
	this->propulsiveLinearEffortZ_percent.setValue(source.getPropulsiveLinearEffortZ_percent()); 
 
	this->propulsiveRotationalEffortX_percent.setName("PropulsiveEffort");
	this->propulsiveRotationalEffortX_percent.setOptional(false);
	this->propulsiveRotationalEffortX_percent.setValue(source.getPropulsiveRotationalEffortX_percent()); 
 
	this->propulsiveRotationalEffortY_percent.setName("PropulsiveEffort");
	this->propulsiveRotationalEffortY_percent.setOptional(false);
	this->propulsiveRotationalEffortY_percent.setValue(source.getPropulsiveRotationalEffortY_percent()); 
 
	this->propulsiveRotationalEffortZ_percent.setName("PropulsiveEffort");
	this->propulsiveRotationalEffortZ_percent.setOptional(false);
	this->propulsiveRotationalEffortZ_percent.setValue(source.getPropulsiveRotationalEffortZ_percent()); 
 
	this->resistiveLinearEffortX_percent.setName("ResistiveEffort");
	this->resistiveLinearEffortX_percent.setOptional(false);
	this->resistiveLinearEffortX_percent.setValue(source.getResistiveLinearEffortX_percent()); 
 
	this->resistiveLinearEffortY_percent.setName("ResistiveEffort");
	this->resistiveLinearEffortY_percent.setOptional(false);
	this->resistiveLinearEffortY_percent.setValue(source.getResistiveLinearEffortY_percent()); 
 
	this->resistiveLinearEffortZ_percent.setName("ResistiveEffort");
	this->resistiveLinearEffortZ_percent.setOptional(false);
	this->resistiveLinearEffortZ_percent.setValue(source.getResistiveLinearEffortZ_percent()); 
 
	this->resistiveRotationalEffortX_percent.setName("ResistiveEffort");
	this->resistiveRotationalEffortX_percent.setOptional(false);
	this->resistiveRotationalEffortX_percent.setValue(source.getResistiveRotationalEffortX_percent()); 
 
	this->resistiveRotationalEffortY_percent.setName("ResistiveEffort");
	this->resistiveRotationalEffortY_percent.setOptional(false);
	this->resistiveRotationalEffortY_percent.setValue(source.getResistiveRotationalEffortY_percent()); 
 
	this->resistiveRotationalEffortZ_percent.setName("ResistiveEffort");
	this->resistiveRotationalEffortZ_percent.setOptional(false);
	this->resistiveRotationalEffortZ_percent.setValue(source.getResistiveRotationalEffortZ_percent()); 
 
}

} // namespace mobility
} // namespace openjaus

