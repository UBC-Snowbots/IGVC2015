/**
\file Subsystem.h

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

#include "openjaus/model/Subsystem.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Subsystem::Subsystem()
{
}

Subsystem::Subsystem(const Subsystem& copy)
{
	name = copy.name;
	id = copy.id;

	for(std::map< unsigned char, Node * >::const_iterator i = copy.nodes.begin(); i != copy.nodes.end(); ++i)
	{
		Node *newNode = new Node(*i->second);
		nodes[newNode->getId()] = newNode;
	}

}
// End of user code

// Start of user code for default destructor:
Subsystem::~Subsystem()
{
	for(std::map<unsigned char, Node*>::iterator i = nodes.begin();
		i != nodes.end();
		++i)
	{
		delete i->second;
	}

}
// End of user code

std::string Subsystem::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool Subsystem::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


int Subsystem::getId() const
{
	// Start of user code for accessor getId:
	
	return id;
	// End of user code
}

bool Subsystem::setId(int id)
{
	// Start of user code for accessor setId:
	this->id = id;
	return true;
	// End of user code
}


const std::map< unsigned char, Node * >& Subsystem::getNodes() const
{
	// Start of user code for accessor getNodes:
	
	return nodes;
	// End of user code
}



// Class Methods
bool Subsystem::addNode(Node *node)
{
	// Start of user code for method addNode:
	if(nodes.count(node->getId()))
	{
		return false;
	}

	node->setSubsystem(this);
	nodes[node->getId()] = node;

	return true;
	// End of user code
}




std::string Subsystem::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "Subsystem: " << id << ", \"" << name << "\"\n";
	for(std::map<unsigned char, Node *>::const_iterator i = nodes.begin();
		i != nodes.end();
		++i)
	{
		oss << "\t" << i->second->toString() << "\n";
	}
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Subsystem& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
std::string Subsystem::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<subsystem id=\"" << id << "\" name=\"" << name << "\">\n";
	for(std::map<unsigned char, Node *>::const_iterator i = nodes.begin();
		i != nodes.end();
		++i)
	{
		oss << i->second->toXml(level+1);
	}
	oss << prefix.str() << "</subsystem>\n";
	return oss.str();
}

Json::Value Subsystem::toJson() const
{
	Json::Value root;
	root["name"] = this->name;
	root["id"] = this->id;
	root["nodes"] = Json::Value(Json::arrayValue);

	for(std::map<unsigned char, Node *>::const_iterator i = nodes.begin();
		i != nodes.end();
		++i)
	{
		root["nodes"].append(i->second->toJson());
	}

	return root;
}

std::vector<transport::Address> Subsystem::lookupService(std::string uri, int versionMajor, int versionMinor, transport::Address& addressHint)
{
	std::vector<transport::Address> addresses;

	std::map<unsigned char, Node*>::iterator i;
	if(addressHint.getNode() == transport::Address::ANY_NODE)
	{
		i = nodes.begin();
		while(i != nodes.end())
		{
			std::vector<transport::Address> additions = i->second->lookupService(uri, versionMajor, versionMinor, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
			++i;
		}
	}
	else
	{
		i = nodes.find(addressHint.getNode());
		if(i != nodes.end())
		{
			std::vector<transport::Address> additions = i->second->lookupService(uri, versionMajor, versionMinor, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
		}
	}

	return addresses;
}

std::vector<transport::Address> Subsystem::lookupComponent(std::string name, transport::Address& addressHint)
{
	std::vector<transport::Address> addresses;

	std::map<unsigned char, Node*>::iterator i;
	if(addressHint.getNode() == transport::Address::ANY_NODE)
	{
		i = nodes.begin();
		while(i != nodes.end())
		{
			std::vector<transport::Address> additions = i->second->lookupComponent(name, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
			++i;
		}
	}
	else
	{
		i = nodes.find(addressHint.getNode());
		if(i != nodes.end())
		{
			std::vector<transport::Address> additions = i->second->lookupComponent(name, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
		}
	}

	return addresses;
}
// End of user code

} // namespace model
} // namespace openjaus

