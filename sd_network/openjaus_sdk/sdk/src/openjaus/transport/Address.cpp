/**
\file Address.h

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

#include "openjaus/transport/Address.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{

// Start of user code for default constructor:
Address::Address() :
		subsystem(0),
		node(0),
		component(0)
{
}

Address::Address(uint16_t subsystem, unsigned char node, unsigned char component) :
		subsystem(subsystem),
		node(node),
		component(component)
{
}
// End of user code

// Start of user code for default destructor:
Address::~Address()
{
}
// End of user code

uint16_t Address::getSubsystem() const
{
	// Start of user code for accessor getSubsystem:
	
	return subsystem;
	// End of user code
}

bool Address::setSubsystem(uint16_t subsystem)
{
	// Start of user code for accessor setSubsystem:
	this->subsystem = subsystem;
	return true;
	// End of user code
}


unsigned char Address::getNode() const
{
	// Start of user code for accessor getNode:
	
	return node;
	// End of user code
}

bool Address::setNode(unsigned char node)
{
	// Start of user code for accessor setNode:
	this->node = node;
	return true;
	// End of user code
}


unsigned char Address::getComponent() const
{
	// Start of user code for accessor getComponent:
	
	return component;
	// End of user code
}

bool Address::setComponent(unsigned char component)
{
	// Start of user code for accessor setComponent:
	this->component = component;
	return true;
	// End of user code
}


int Address::getHash() const
{
	// Start of user code for accessor getHash:
	return (subsystem << 16) + (node << 8) + component;
	// End of user code
}



// Class Methods
bool Address::isValid()
{
	// Start of user code for method isValid:
	if(subsystem == THIS_SUBSYSTEM || subsystem == ANY_SUBSYSTEM)
	{
		return false;
	}

	if(node == THIS_NODE|| node == ANY_NODE)
	{
		return false;
	}

	if(component == THIS_COMPONENT || component == ANY_COMPONENT)
	{
		return false;
	}

	return true;
	// End of user code
}



int Address::to(system::Buffer *dst)
{
	// Start of user code for method to:
	uint32_t address = (subsystem << 16) + (node << 8) + component;
	return 	dst->pack(address);
	// End of user code
}

int Address::from(system::Buffer *src)
{
	// Start of user code for method from:
	uint32_t address;
	int length = src->unpack(address);
	subsystem = (address >> 16) & 0xFFFF;
	node = (address >> 8) & 0xFF;
	component = address & 0xFF;
	return 	length;
	// End of user code
}

int Address::length()
{
	// Start of user code for method length:
	return sizeof(subsystem) + sizeof(node) + sizeof(component);
	// End of user code
}


std::string Address::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "JAUS Address: " << static_cast<unsigned short>(subsystem) << "." << static_cast<unsigned int>(node) << "." << static_cast<unsigned int>(component);
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Address& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
Json::Value Address::toJson() const
{
	Json::Value root;
	root["subsystem"] = static_cast<unsigned short>(subsystem);
	root["node"] = static_cast<unsigned short>(node);
	root["component"] = static_cast<unsigned short>(component);
	return root;
}

std::string Address::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<address";
	oss << " subsystem=\"" << static_cast<unsigned short>(subsystem) << "\"";
	oss << " node=\"" << static_cast<unsigned short>(node) << "\"";
	oss << " component=\"" << static_cast<unsigned short>(component) << "\"";
	oss << " />\n";
	return oss.str();
}

bool Address::operator==(const Address &other) const
{
	if(this->getHash() == other.getHash())
	{
		return true;
	}
	return false;
}

bool Address::operator!=(const Address &other) const
{
	return !(*this == other);
}

bool Address::operator<(const Address &other) const
{
	if(this->getHash() < other.getHash())
	{
		return true;
	}
	return false;
}

// End of user code

} // namespace transport
} // namespace openjaus

