/**
\file Component.h

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

#include "openjaus/model/Component.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
Component::Component() :
	name(),
	id(0),
	services(),
	authority(DEFAULT_AUTHORITY),
	address(),
	runners(),
	interfaces(),
	node(NULL),
	systemTree(NULL),
	implements(new std::vector<Service*>()),
	inheritsFrom(NULL)
{
}

Component::Component(const Component& copy) :
	name(copy.name),
	id(copy.id),
	services(),
	address(copy.address),
	runners(),
	interfaces(),
	node(NULL),
	systemTree(NULL),
	implements(new std::vector<Service*>()),
	inheritsFrom(NULL)
{
	services.clear();
	for(std::map< std::string, model::Service * >::const_iterator i = copy.services.begin(); i != copy.services.end(); ++i)
	{
		model::Service *newService = new model::Service(*i->second);
		services[newService->getUri()] = newService;
	}
}
// End of user code

// Start of user code for default destructor:
Component::~Component()
{
	for(size_t i = 0; i < implements->size(); ++i)
	{
		if(implements->at(i))
		{
			delete implements->at(i);
		}
	}
	delete this->implements;

	clearServices();
}
// End of user code

std::string Component::getName() const
{
	// Start of user code for accessor getName:
	
	return name;
	// End of user code
}

bool Component::setName(std::string name)
{
	// Start of user code for accessor setName:
	this->name = name;
	return true;
	// End of user code
}


int Component::getId() const
{
	// Start of user code for accessor getId:
	
	return id;
	// End of user code
}

bool Component::setId(int id)
{
	// Start of user code for accessor setId:
	this->id = id;
	return true;
	// End of user code
}


const std::map< std::string, Service * >& Component::getServices() const
{
	// Start of user code for accessor getServices:
	
	return services;
	// End of user code
}


uint8_t Component::getAuthority() const
{
	// Start of user code for accessor getAuthority:
	
	return authority;
	// End of user code
}

bool Component::setAuthority(uint8_t authority)
{
	// Start of user code for accessor setAuthority:
	this->authority = authority;
	return true;
	// End of user code
}


const transport::Address& Component::getAddress() const
{
	// Start of user code for accessor getAddress:
	
	return address;
	// End of user code
}

bool Component::setAddress(const transport::Address& address)
{
	// Start of user code for accessor setAddress:
	this->address = address;
	return true;
	// End of user code
}


const std::vector< StateMachineRunner* >& Component::getRunners() const
{
	// Start of user code for accessor getRunners:
	
	return runners;
	// End of user code
}

bool Component::setRunners(const StateMachineRunner& runners)
{
	// Start of user code for accessor setRunners:
	return true;
	// End of user code
}


const std::vector< transport::Interface* >& Component::getInterfaces() const
{
	// Start of user code for accessor getInterfaces:
	
	return interfaces;
	// End of user code
}

bool Component::setInterfaces(const transport::Interface& interfaces)
{
	// Start of user code for accessor setInterfaces:
	return true;
	// End of user code
}


Node* Component::getNode() const
{
	// Start of user code for accessor getNode:
	
	return node;
	// End of user code
}

bool Component::setNode(Node* node)
{
	// Start of user code for accessor setNode:
	this->node = node;
	return true;
	// End of user code
}


SystemTree* Component::getSystemTree() const
{
	// Start of user code for accessor getSystemTree:
	
	return systemTree;
	// End of user code
}

bool Component::setSystemTree(SystemTree* systemTree)
{
	// Start of user code for accessor setSystemTree:
	this->systemTree = systemTree;
	return true;
	// End of user code
}


std::vector< Service* >* Component::getImplements() const
{
	// Start of user code for accessor getImplements:
	
	return implements;
	// End of user code
}

bool Component::setImplements(Service* implements)
{
	// Start of user code for accessor setImplements:
	return true;
	// End of user code
}


Component* Component::getInheritsFrom() const
{
	// Start of user code for accessor getInheritsFrom:
	
	return inheritsFrom;
	// End of user code
}

bool Component::setInheritsFrom(Component* inheritsFrom)
{
	// Start of user code for accessor setInheritsFrom:
	this->inheritsFrom = inheritsFrom;
	return true;
	// End of user code
}



// Class Methods
bool Component::addService(Service *service)
{
	// Start of user code for method addService:
	if(services.find(service->getUri()) == services.end())
	{
		this->services[service->getUri()] = service;
		return true;
	}

	return false;
	// End of user code
}


void Component::run()
{
	// Start of user code for method run:
	for(unsigned int i = 0; i < runners.size(); ++i)
	{
		runners[i]->create();
	}

	for(unsigned int i = 0; i < interfaces.size(); ++i)
	{
		interfaces[i]->run();
	}

	// End of user code
}


void Component::stop()
{
	// Start of user code for method stop:
	for(unsigned int i = 0; i < interfaces.size(); ++i)
	{
		interfaces[i]->stop();
	}

	for(unsigned int i = 0; i < runners.size(); ++i)
	{
		runners[i]->stop();
		runners[i]->join();
	}
	// End of user code
}




std::string Component::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "Component: " << id << ", \"" << name << "\"\n";
	oss << "\t\t\t" << "Address: " << this->address.toString() << "\n";

	std::map< std::string, model::Service * >::const_iterator iter;
	for(iter = this->services.begin(); iter != this->services.end(); ++iter)
	{
		oss << "\t\t\t" << iter->second->toString();
	}

	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Component& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
void Component::clearServices()
{
	for(std::map< std::string, model::Service * >::const_iterator i = services.begin(); i != services.end(); ++i)
	{
		if(i->second)
		{
			delete i->second;
		}
	}
	this->services.clear();
}

Json::Value Component::toJson() const
{
	Json::Value root;
	root["name"] = this->name;
	root["id"] = this->id;
	root["address"] = this->address.toString();
	root["services"] = Json::Value(Json::arrayValue);

	std::map< std::string, model::Service * >::const_iterator iter;
	for(iter = this->services.begin(); iter != this->services.end(); ++iter)
	{
		root["services"].append(iter->second->toJson());
	}

	return root;
}

std::string Component::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<component id=\"" << id << "\" name=\"" << name << "\">\n";
	oss << this->address.toXml(level+1);
	std::map< std::string, model::Service * >::const_iterator iter;
	for(iter = this->services.begin(); iter != this->services.end(); ++iter)
	{
		oss << iter->second->toXml(level+1);
	}
	oss << prefix.str() << "</component>\n";
	return oss.str();
}

bool Component::containsService(std::string uri, int versionMajor, int versionMinor)
{
	std::map<std::string, Service* >::iterator i = services.find(uri);
	if(i == services.end())
	{
		return false;
	}

	// TODO: Check for multiple versions of same service uri

	if(versionMajor != Service::ANY_VERSION && i->second->getVersionMajor() != versionMajor)
	{
		return false;
	}

	if(versionMinor != Service::ANY_VERSION && i->second->getVersionMinor() != versionMinor)
	{
		return false;
	}

	return true;
}
// End of user code

} // namespace model
} // namespace openjaus

