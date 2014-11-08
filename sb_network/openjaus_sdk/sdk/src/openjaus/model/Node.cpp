/**
\file Node.h

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

#include "openjaus/model/Node.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Node::Node()
{
}

Node::Node(const Node& copy)
{
	name = copy.name;
	id = copy.id;
	subsystem = NULL;

	for(std::map< unsigned char, Component * >::const_iterator i = copy.components.begin(); i != copy.components.end(); ++i)
	{
		Component *newComponent = new Component(*i->second);
		components[newComponent->getId()] = newComponent;
	}
}
// End of user code

// Start of user code for default destructor:
Node::~Node()
{
	for(std::map<unsigned char, Component*>::iterator i = components.begin();
		i != components.end();
		++i)
	{
		delete i->second;
	}
}
// End of user code

std::string Node::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool Node::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


int Node::getId() const
{
	// Start of user code for accessor getId:
	
	return id;
	// End of user code
}

bool Node::setId(int id)
{
	// Start of user code for accessor setId:
	this->id = id;
	return true;
	// End of user code
}


const std::map< unsigned char, Component * >& Node::getComponents() const
{
	// Start of user code for accessor getComponents:
	
	return components;
	// End of user code
}


Subsystem* Node::getSubsystem() const
{
	// Start of user code for accessor getSubsystem:
	
	return subsystem;
	// End of user code
}

bool Node::setSubsystem(Subsystem* subsystem)
{
	// Start of user code for accessor setSubsystem:
	this->subsystem = subsystem;
	return true;
	// End of user code
}



// Class Methods
bool Node::addComponent(Component *component)
{
	// Start of user code for method addComponent:

	if(components.count(component->getId()))
	{
		return false;
	}

	component->setNode(this);
	components[component->getId()] = component;

	return true;

	// End of user code
}




std::string Node::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "Node: " << id << ", \"" << name << "\"\n";
	for(std::map<unsigned char, Component *>::const_iterator i = components.begin();
		i != components.end();
		++i)
	{
		oss << "\t\t" << i->second->toString();
	}
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Node& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
std::string Node::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<node id=\"" << id << "\" name=\"" << name << "\">\n";
	for(std::map<unsigned char, Component *>::const_iterator i = components.begin();
		i != components.end();
		++i)
	{
		oss << i->second->toXml(level+1);
	}
	oss << prefix.str() << "</node>\n";
	return oss.str();
}


Json::Value Node::toJson() const
{
	Json::Value root;
	root["name"] = this->name;
	root["id"] = this->id;
	root["components"] = Json::Value(Json::arrayValue);

	for(std::map<unsigned char, Component *>::const_iterator i = components.begin();
		i != components.end();
		++i)
	{
		root["components"].append(i->second->toJson());
	}

	return root;
}

std::vector<transport::Address> Node::lookupService(std::string uri, int versionMajor, int versionMinor, transport::Address& addressHint)
{
	std::vector<transport::Address> addresses;

	std::map<unsigned char, Component*>::iterator i;
	if(addressHint.getComponent() == transport::Address::ANY_COMPONENT)
	{
		i = components.begin();
		while(i != components.end())
		{
			if(i->second->containsService(uri, versionMajor, versionMinor))
			{
				addresses.push_back(i->second->getAddress());
			}
			++i;
		}
	}
	else
	{
		i = components.find(addressHint.getComponent());
		if(i != components.end() && i->second->containsService(uri, versionMajor, versionMinor))
		{
			addresses.push_back(i->second->getAddress());
		}
	}

	return addresses;
}

std::vector<transport::Address> Node::lookupComponent(std::string name, transport::Address& addressHint)
{
	std::vector<transport::Address> addresses;

	std::map<unsigned char, Component*>::iterator i;
	if(addressHint.getComponent() == transport::Address::ANY_COMPONENT)
	{
		i = components.begin();
		while(i != components.end())
		{
			if(i->second->getName().compare(name) == 0)
			{
				addresses.push_back(i->second->getAddress());
			}
			++i;
		}
	}
	else
	{
		i = components.find(addressHint.getComponent());
		if(i != components.end() && i->second->getName().compare(name) == 0)
		{
			addresses.push_back(i->second->getAddress());
		}
	}

	return addresses;
}
// End of user code

} // namespace model
} // namespace openjaus

