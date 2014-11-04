

/**
\file AddressBitField.h

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
#include "openjaus/core/Triggers/Fields/AddressBitField.h"

namespace openjaus
{
namespace core
{

AddressBitField::AddressBitField() : 
	subsystem(),
	node(),
	component()
{

}

AddressBitField::~AddressBitField()
{

}

bool AddressBitField::setSubsystem(long value)
{
	if(value < SUBSYSTEM_MIN_VALUE || value > SUBSYSTEM_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->subsystem = value;
	return true;
}
    
long AddressBitField::getSubsystem(void) const
{
	return 	this->subsystem;
}

bool AddressBitField::setNode(long value)
{
	if(value < NODE_MIN_VALUE || value > NODE_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->node = value;
	return true;
}
    
long AddressBitField::getNode(void) const
{
	return 	this->node;
}

bool AddressBitField::setComponent(long value)
{
	if(value < COMPONENT_MIN_VALUE || value > COMPONENT_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->component = value;
	return true;
}
    
long AddressBitField::getComponent(void) const
{
	return 	this->component;
}


int AddressBitField::to(system::Buffer *dst)
{
	uint32_t intValue = 0;

	intValue |= ((this->subsystem & AddressBitField::SUBSYSTEM_BIT_MASK) << AddressBitField::SUBSYSTEM_START_BIT);
	intValue |= ((this->node & AddressBitField::NODE_BIT_MASK) << AddressBitField::NODE_START_BIT);
	intValue |= ((this->component & AddressBitField::COMPONENT_BIT_MASK) << AddressBitField::COMPONENT_START_BIT);
	return dst->pack(intValue);
}

int AddressBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint32_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->subsystem = (intValue >> (AddressBitField::SUBSYSTEM_START_BIT)) & AddressBitField::SUBSYSTEM_BIT_MASK;
	this->node = (intValue >> (AddressBitField::NODE_START_BIT)) & AddressBitField::NODE_BIT_MASK;
	this->component = (intValue >> (AddressBitField::COMPONENT_START_BIT)) & AddressBitField::COMPONENT_BIT_MASK;

	return byteSize;
}

int AddressBitField::length(void)
{
	return sizeof(uint32_t);
}

void AddressBitField::copy(AddressBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setSubsystem(source.getSubsystem());
	setNode(source.getNode());
	setComponent(source.getComponent());

}

std::string AddressBitField::toXml(unsigned char level) const
{
	uint32_t intValue = 0;

	intValue |= ((this->subsystem & AddressBitField::SUBSYSTEM_BIT_MASK) << AddressBitField::SUBSYSTEM_START_BIT);
	intValue |= ((this->node & AddressBitField::NODE_BIT_MASK) << AddressBitField::NODE_START_BIT);
	intValue |= ((this->component & AddressBitField::COMPONENT_BIT_MASK) << AddressBitField::COMPONENT_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"subsystem\" value=\"" << getSubsystem() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"node\" value=\"" << getNode() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"component\" value=\"" << getComponent() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace core
} // namespace openjaus

