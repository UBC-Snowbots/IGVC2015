

/**
\file ParentIDBitField.h

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
#include "openjaus/manipulator/Triggers/Fields/ParentIDBitField.h"

namespace openjaus
{
namespace manipulator
{

ParentIDBitField::ParentIDBitField() : 
	componentID(),
	nodeID(),
	subsystemID()
{

}

ParentIDBitField::~ParentIDBitField()
{

}

bool ParentIDBitField::setComponentID(long value)
{
	if(value < COMPONENTID_MIN_VALUE || value > COMPONENTID_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->componentID = value;
	return true;
}
    
long ParentIDBitField::getComponentID(void) const
{
	return 	this->componentID;
}

bool ParentIDBitField::setNodeID(long value)
{
	if(value < NODEID_MIN_VALUE || value > NODEID_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->nodeID = value;
	return true;
}
    
long ParentIDBitField::getNodeID(void) const
{
	return 	this->nodeID;
}

bool ParentIDBitField::setSubsystemID(long value)
{
	if(value < SUBSYSTEMID_MIN_VALUE || value > SUBSYSTEMID_MAX_VALUE)
	{
		//< \todo: Throw error?
		return false;
	}
	
	this->subsystemID = value;
	return true;
}
    
long ParentIDBitField::getSubsystemID(void) const
{
	return 	this->subsystemID;
}


int ParentIDBitField::to(system::Buffer *dst)
{
	uint32_t intValue = 0;

	intValue |= ((this->componentID & ParentIDBitField::COMPONENTID_BIT_MASK) << ParentIDBitField::COMPONENTID_START_BIT);
	intValue |= ((this->nodeID & ParentIDBitField::NODEID_BIT_MASK) << ParentIDBitField::NODEID_START_BIT);
	intValue |= ((this->subsystemID & ParentIDBitField::SUBSYSTEMID_BIT_MASK) << ParentIDBitField::SUBSYSTEMID_START_BIT);
	return dst->pack(intValue);
}

int ParentIDBitField::from(system::Buffer *src)
{
	int byteSize = 0;
	uint32_t intValue = 0;
	byteSize = src->unpack(intValue);

	this->componentID = (intValue >> (ParentIDBitField::COMPONENTID_START_BIT)) & ParentIDBitField::COMPONENTID_BIT_MASK;
	this->nodeID = (intValue >> (ParentIDBitField::NODEID_START_BIT)) & ParentIDBitField::NODEID_BIT_MASK;
	this->subsystemID = (intValue >> (ParentIDBitField::SUBSYSTEMID_START_BIT)) & ParentIDBitField::SUBSYSTEMID_BIT_MASK;

	return byteSize;
}

int ParentIDBitField::length(void)
{
	return sizeof(uint32_t);
}

void ParentIDBitField::copy(ParentIDBitField& source)
{
	this->setName(source.getName());
	this->setInterpretation(source.getInterpretation());
	this->setOptional(source.isOptional());

	setComponentID(source.getComponentID());
	setNodeID(source.getNodeID());
	setSubsystemID(source.getSubsystemID());

}

std::string ParentIDBitField::toXml(unsigned char level) const
{
	uint32_t intValue = 0;

	intValue |= ((this->componentID & ParentIDBitField::COMPONENTID_BIT_MASK) << ParentIDBitField::COMPONENTID_START_BIT);
	intValue |= ((this->nodeID & ParentIDBitField::NODEID_BIT_MASK) << ParentIDBitField::NODEID_START_BIT);
	intValue |= ((this->subsystemID & ParentIDBitField::SUBSYSTEMID_BIT_MASK) << ParentIDBitField::SUBSYSTEMID_START_BIT);

	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<BitField name=\"" << this->name << "\">\n";
	oss << prefix.str() << "\t" << "<intValue>" << intValue << "</intValue>\n";
	oss << prefix.str() << "\t" << "<fields>\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"ComponentID\" value=\"" << getComponentID() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"NodeID\" value=\"" << getNodeID() << "\" />\n";
	oss << prefix.str() << "\t" << "<BitFieldRange name=\"SubsystemID\" value=\"" << getSubsystemID() << "\" />\n";
	oss << prefix.str() << "\t" << "</fields>\n";
	oss << prefix.str() << "</BitField>\n";
	return oss.str();
}

} // namespace manipulator
} // namespace openjaus

