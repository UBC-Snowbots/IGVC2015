/**
\file SystemTree.h

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

#include "openjaus/model/SystemTree.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"
#include "openjaus/system/ScopeLock.h"
// End of user code

namespace openjaus
{
namespace model
{

// Start of user code for default constructor:
SystemTree::SystemTree() :
		subsystems(),
		thisSubsytem(0),
		thisNode(0),
		mutex()
{
}
// End of user code

// Start of user code for default destructor:
SystemTree::~SystemTree()
{
	for(std::map<short, Subsystem*>::iterator i = subsystems.begin();
		i != subsystems.end();
		++i)
	{
		delete i->second;
	}

}
// End of user code

const std::map< short, Subsystem * >& SystemTree::getSubsystems() const
{
	// Start of user code for accessor getSubsystems:
	return subsystems;
	// End of user code
}


uint16_t SystemTree::getThisSubsytem() const
{
	// Start of user code for accessor getThisSubsytem:
	return thisSubsytem;
	// End of user code
}

bool SystemTree::setThisSubsytem(uint16_t thisSubsytem)
{
	// Start of user code for accessor setThisSubsytem:
	if(this->thisSubsytem && this->thisSubsytem != thisSubsytem)
	{
		THROW_EXCEPTION("Set subsystem conflicts existing value: " << thisSubsytem << " != " << this->thisSubsytem);
	}
	this->thisSubsytem = thisSubsytem;
	return true;
	// End of user code
}


uint8_t SystemTree::getThisNode() const
{
	// Start of user code for accessor getThisNode:
	return thisNode;
	// End of user code
}

bool SystemTree::setThisNode(uint8_t thisNode)
{
	// Start of user code for accessor setThisNode:
	if(this->thisNode && this->thisNode != thisNode)
	{
		THROW_EXCEPTION("Set node conflicts existing value: " << thisNode << " != " << this->thisNode);
	}
	this->thisNode = thisNode;
	return true;
	// End of user code
}


const system::Mutex& SystemTree::getMutex() const
{
	// Start of user code for accessor getMutex:
	
	return mutex;
	// End of user code
}



// Class Methods

bool SystemTree::hasComponent(short subsystemId, unsigned char nodeId, unsigned char componentId)
{
	// Start of user code for method hasComponent(short subsystemId, unsigned char nodeId, unsigned char componentId):
	if(findComponent(subsystemId, nodeId, componentId))
	{
		return true;
	}

	return false;
	// End of user code
}



bool SystemTree::hasNode(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method hasNode(short subsystemId, unsigned char nodeId):
	if(findNode(subsystemId, nodeId))
	{
		return true;
	}

	return false;
	// End of user code
}



bool SystemTree::hasSubsystem(short subsystemId)
{
	// Start of user code for method hasSubsystem(short subsystemId):
	if(subsystems.count(subsystemId))
	{
		return true;
	}

	return false;
	// End of user code
}


bool SystemTree::hasComponentIdentification(Component &component)
{
	// Start of user code for method hasComponentIdentification(Component &component):
	return hasComponentIdentification(component.getAddress());
	// End of user code
}



bool SystemTree::hasComponentIdentification(short subsystemId, unsigned char nodeId, unsigned char componentId)
{
	// Start of user code for method hasComponentIdentification(short subsystemId, unsigned char nodeId, unsigned char componentId):
	Component *cmpt = findComponent(subsystemId, nodeId, componentId);
	if(!cmpt)
	{
		THROW_EXCEPTION("hasComponentIdentification: Could not find component: " << static_cast<unsigned int>(subsystemId) <<
						"." << static_cast<unsigned int>(nodeId) <<
						"." << static_cast<unsigned int>(componentId));
	}

	if(cmpt->getName().length())
	{
		return true;
	}

	return false;
	// End of user code
}


bool SystemTree::hasNodeIdentification(Node &node)
{
	// Start of user code for method hasNodeIdentification(Node &node):
	return hasNodeIdentification(node.getSubsystem()->getId(), node.getId());
	// End of user code
}



bool SystemTree::hasNodeIdentification(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method hasNodeIdentification(short subsystemId, unsigned char nodeId):
	Node *node = findNode(subsystemId, nodeId);
	if(!node)
	{
		THROW_EXCEPTION("hasNodeIdentification: Could not find node: " << static_cast<unsigned int>(subsystemId) <<
						"." << static_cast<unsigned int>(nodeId));
	}

	if(node->getName().length())
	{
		return true;
	}

	return false;
	// End of user code
}


bool SystemTree::hasSubsystemIdentification(Subsystem &subsystem)
{
	// Start of user code for method hasSubsystemIdentification(Subsystem &subsystem):
	return hasSubsystemIdentification(subsystem.getId());
	// End of user code
}



bool SystemTree::hasSubsystemIdentification(short subsystemId)
{
	// Start of user code for method hasSubsystemIdentification(short subsystemId):
	if(!subsystems.count(subsystemId))
	{
		THROW_EXCEPTION("hasSubsystemIdentification: Could not find subsystem: " << static_cast<unsigned int>(subsystemId));
	}

	Subsystem *subs = subsystems[subsystemId];
	if(subs->getName().length())
	{
		return true;
	}

	return false;
	// End of user code
}


bool SystemTree::hasComponentConfiguration(Component &component)
{
	// Start of user code for method hasComponentConfiguration(Component &component):
	return hasComponentConfiguration(component.getAddress());
	// End of user code
}



bool SystemTree::hasComponentConfiguration(short subsystemId, unsigned char nodeId, unsigned char componentId)
{
	// Start of user code for method hasComponentConfiguration(short subsystemId, unsigned char nodeId, unsigned char componentId):
	Component *cmpt = findComponent(subsystemId, nodeId, componentId);
	if(!cmpt)
	{
		THROW_EXCEPTION("hasComponentConfiguration: Could not find component: " << static_cast<unsigned int>(subsystemId) <<
						"." << static_cast<unsigned int>(nodeId) <<
						"." << static_cast<unsigned int>(componentId));
	}

	if(cmpt->getServices().size())
	{
		return true;
	}

	return false;

	// End of user code
}


bool SystemTree::hasNodeConfiguration(Node &node)
{
	// Start of user code for method hasNodeConfiguration(Node &node):
	return hasNodeConfiguration(node.getSubsystem()->getId(), node.getId());
	// End of user code
}



bool SystemTree::hasNodeConfiguration(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method hasNodeConfiguration(short subsystemId, unsigned char nodeId):
	Node *node = findNode(subsystemId, nodeId);
	if(!node)
	{
		THROW_EXCEPTION("hasNodeConfiguration: Could not find node: " << static_cast<unsigned int>(subsystemId) <<
						"." << static_cast<unsigned int>(nodeId));
	}

	if(node->getComponents().size())
	{
		return true;
	}

	return false;
	// End of user code
}


bool SystemTree::hasSubsystemConfiguration(Subsystem &subsystem)
{
	// Start of user code for method hasSubsystemConfiguration(Subsystem &subsystem):
	return hasSubsystemConfiguration(subsystem.getId());
	// End of user code
}



bool SystemTree::hasSubsystemConfiguration(short subsystemId)
{
	// Start of user code for method hasSubsystemConfiguration(short subsystemId):
	if(!subsystems.count(subsystemId))
	{
		THROW_EXCEPTION("hasSubsystemConfiguration: Could not find subsystem: " << static_cast<unsigned int>(subsystemId));
	}

	Subsystem *subs = subsystems[subsystemId];
	if(subs->getNodes().size())
	{
		return true;
	}

	return false;
	// End of user code
}


Component SystemTree::getComponent(Component &component)
{
	// Start of user code for method getComponent(Component &component):
	transport::Address address = component.getAddress();
	return getComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	// End of user code
}


Component SystemTree::getComponent(const transport::Address &address)
{
	// Start of user code for method getComponent(const transport::Address &address):
	return getComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	// End of user code
}


Component SystemTree::getComponent(short subsystemId, unsigned char nodeId, unsigned char componentId)
{
	// Start of user code for method getComponent(short subsystemId, unsigned char nodeId, unsigned char componentId):
	Component *component = findComponent(subsystemId, nodeId, componentId);
	if(!component)
	{
		THROW_EXCEPTION("SystemTree: Attempted to get non-existent component: (" << subsystemId << "," << nodeId << "," << componentId << ")");
	}

	return Component(*component);
	// End of user code
}


Node SystemTree::getNode(Node &node)
{
	// Start of user code for method getNode(Node &node):
	return getNode(node.getSubsystem()->getId(), node.getId());
	// End of user code
}


Node SystemTree::getNode(const transport::Address &address)
{
	// Start of user code for method getNode(const transport::Address &address):
	return getNode(address.getSubsystem(), address.getNode());
	// End of user code
}


Node SystemTree::getNode(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method getNode(short subsystemId, unsigned char nodeId):
	Node *node = findNode(subsystemId, nodeId);
	if(!node)
	{
		THROW_EXCEPTION("SystemTree: Attempted to get non-existent node: (" << subsystemId << "," << nodeId << ")");
	}

	return Node(*node);
	// End of user code
}


Subsystem SystemTree::getSubsystem(Subsystem &subsystem)
{
	// Start of user code for method getSubsystem(Subsystem &subsystem):
	return getSubsystem(subsystem.getId());
	// End of user code
}


Subsystem SystemTree::getSubsystem(const transport::Address &address)
{
	// Start of user code for method getSubsystem(const transport::Address &address):
	return getSubsystem(address.getSubsystem());
	// End of user code
}


Subsystem SystemTree::getSubsystem(short subsystemId)
{
	// Start of user code for method getSubsystem(short subsystemId):
	if(!subsystems.count(subsystemId) && !subsystems[subsystemId])
	{
		THROW_EXCEPTION("SystemTree: Attempted to get non-existent subsystem: (" << subsystemId << ")");
	}
	return Subsystem(*subsystems[subsystemId]);
	// End of user code
}


bool SystemTree::addService(transport::Address address, std::string uri, int versionMajor, int versionMinor)
{
	// Start of user code for method addService:
	Component *component = findComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	if(!component)
	{
		THROW_EXCEPTION("SystemTree: Attempted to add service to non-existent component: " << address.toString());
	}

	std::map< std::string, model::Service * >& services = const_cast<std::map< std::string, model::Service * >&>(component->getServices());

	if( services.find(uri) == services.end() )
	{
		LOG_DEBUG("Adding Service: " << uri << ", to: " << address.toString());
		model::Service *service = new model::Service();

		service->setName(uri);
		service->setUri(uri);
		service->setVersionMajor(versionMajor);
		service->setVersionMinor(versionMinor);

		services[uri] = service;
	}

	return true;
	// End of user code
}


bool SystemTree::addComponent(Component &component)
{
	// Start of user code for method addComponent(Component &component):
	const transport::Address& address = component.getAddress();
	addComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	Component *cmpt = findComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	cmpt->setName(component.getName());

	std::map< std::string, model::Service * >::const_iterator i = component.getServices().begin();
	while(i != component.getServices().end())
	{
		addService(address, i->first, i->second->getVersionMajor(), i->second->getVersionMinor());
		i++;
	}
	return true;
	// End of user code
}



bool SystemTree::addComponent(short subsystemId, unsigned char nodeId, unsigned char componentId)
{
	// Start of user code for method addComponent(short subsystemId, unsigned char nodeId, unsigned char componentId):
	if(hasComponent(subsystemId, nodeId, componentId))
	{
		THROW_EXCEPTION("Attempted to add component: " << static_cast<unsigned int>(subsystemId) <<
						"." << static_cast<unsigned int>(nodeId) <<
						"." << static_cast<unsigned int>(componentId) <<
						" that is already in system tree");
	}
	transport::Address originalAddress(subsystemId, nodeId, componentId);

	Node *node = findNode(subsystemId, nodeId);
	if(!node)
	{
		addNode(subsystemId, nodeId);
		node = findNode(subsystemId, nodeId);
	}

	// Add the component to the node
	Component *cmpt = new Component();
	cmpt->setId(componentId);
	cmpt->setAddress(originalAddress);
	node->addComponent(cmpt);

	return true;
	// End of user code
}


bool SystemTree::addNode(Node &node)
{
	// Start of user code for method addNode(Node &node):
	return addNode(node.getSubsystem()->getId(), node.getId());
	// End of user code
}



bool SystemTree::addNode(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method addNode(short subsystemId, unsigned char nodeId):
	if(hasNode(subsystemId, nodeId))
	{
		THROW_EXCEPTION("Attempted to add node: " << static_cast<unsigned int>(subsystemId) <<
						"." << static_cast<unsigned int>(nodeId) <<
						" that is already in system tree");
	}

	if(!hasSubsystem(subsystemId))
	{
		addSubsystem(subsystemId);
	}

	Node *node = new Node();
	node->setId(nodeId);
	subsystems[subsystemId]->addNode(node);

	// SystemTreeEvent *e = new SystemTreeEvent(SystemTreeEvent::NodeAdded, node);
	// this->handleEvent(e);

	return true;
	// End of user code
}


bool SystemTree::addSubsystem(Subsystem &subsystem)
{
	// Start of user code for method addSubsystem(Subsystem &subsystem):
	return addSubsystem(subsystem.getId());
	// End of user code
}



bool SystemTree::addSubsystem(short subsystemId)
{
	// Start of user code for method addSubsystem(short subsystemId):
	if(hasSubsystem(subsystemId))
	{
		THROW_EXCEPTION("Attempted to add subsystem: " << static_cast<unsigned int>(subsystemId) << " that is already in system tree");
	}

	subsystems[subsystemId] = new Subsystem();
	subsystems[subsystemId]->setId(subsystemId);

	//SystemTreeEvent *e = new SystemTreeEvent(SystemTreeEvent::SubsystemAdded, subs);
	// TODO: signal()
	return true;
	// End of user code
}


bool SystemTree::replaceComponent(Component &component)
{
	// Start of user code for method replaceComponent:
	transport::Address address = component.getAddress();

	Component *oldCmpt = findComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	if(!oldCmpt)
	{
		THROW_EXCEPTION("SystemTree: Attempted to replace to non-existent component: " << address.toString());
	}

	Node *node = oldCmpt->getNode();

	Component* newCmpt = new Component(component);

	removeComponent(address.getSubsystem(), address.getNode(), address.getComponent());

	// Add the component to the node

	newCmpt->setId(address.getComponent());

	// Add cmpt
	node->addComponent(newCmpt);

	newCmpt->setId(component.getId());
	return true;
	// End of user code
}


bool SystemTree::removeComponent(Component &component)
{
	// Start of user code for method removeComponent(Component &component):
	return removeComponent(component.getAddress());
	// End of user code
}



bool SystemTree::removeComponent(short subsystemId, unsigned char nodeId, unsigned char componentId)
{
	// Start of user code for method removeComponent(short subsystemId, unsigned char nodeId, unsigned char componentId):

	Node *node = findNode(subsystemId, nodeId);
	if(!node)
	{
		return false;
	}

	std::map<unsigned char, Component *>& components = const_cast<std::map<unsigned char, Component *>&>(node->getComponents());

	if(!components.count(componentId))
	{
		return false;
	}

	Component *component = components[componentId];
	if(!component)
	{
		return false;
	}

	components.erase(componentId);
	delete component;
	return true;
	// End of user code
}


bool SystemTree::removeNode(Node &node)
{
	// Start of user code for method removeNode(Node &node):
	return removeNode(node.getSubsystem()->getId(), node.getId());
	// End of user code
}



bool SystemTree::removeNode(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method removeNode(short subsystemId, unsigned char nodeId):

	if(!subsystems.count(subsystemId))
	{
		return false;
	}

	std::map<unsigned char, Node *>& nodes = const_cast<std::map<unsigned char, Node *>&>(subsystems[subsystemId]->getNodes());

	if(!nodes.count(nodeId))
	{
		return false;
	}

	Node *node = nodes[nodeId];
	if(!node)
	{
		return false;
	}

	nodes.erase(nodeId);
	delete node;
	return true;
	// End of user code
}


bool SystemTree::removeSubsystem(Subsystem &subsystem)
{
	// Start of user code for method removeSubsystem(Subsystem &subsystem):
	return removeSubsystem(subsystem.getId());
	// End of user code
}



bool SystemTree::removeSubsystem(short subsystemId)
{
	// Start of user code for method removeSubsystem(short subsystemId):

	if(subsystems.count(subsystemId))
	{
		Subsystem *subs = subsystems[subsystemId];
		subsystems.erase(subsystemId);
		delete subs;
		return true;
	}
	return false;
	// End of user code
}


void SystemTree::lock()
{
	// Start of user code for method lock:
	mutex.lock();
	// End of user code
}


void SystemTree::unlock()
{
	// Start of user code for method unlock:
	mutex.unlock();
	// End of user code
}


unsigned char SystemTree::getAvailableComponentId(short subsystemId, unsigned char nodeId)
{
	// Start of user code for method getAvailableComponentId:
	LOG_DEBUG("Getting available component id");

	Node* node = findNode(subsystemId, nodeId);
	if(!node)
	{
		LOG_DEBUG("Could not find node at: " << subsystemId  << "." << (int)nodeId);
	}

	static std::vector<bool> assigned;
	static double lastCheckTime_sec = 0;
	if(lastCheckTime_sec + 1.25 < system::Time::getTime().inSec())
	{
		lastCheckTime_sec = system::Time::getTime().inSec();
		assigned = std::vector<bool>(model::Component::MAXIMUM_ID, false);
		std::map< unsigned char, Component * >::const_iterator i;
		if(node)
		{
			for(i = node->getComponents().begin(); i != node->getComponents().end(); i++)
			{
				assigned[i->first] = true;
			}
		}
	}

	for(int i = model::Component::MINIMUM_ID; i <= model::Component::MAXIMUM_ID; ++i)
	{
		if(!assigned[i])
		{
			LOG_DEBUG("Assigning Component ID: " << i);
			assigned[i] = true;
			return static_cast<unsigned char>(i);
		}
	}

	THROW_EXCEPTION("No component IDs available on local node");
	// End of user code
}


unsigned char SystemTree::getAvailableNodeId(short subsystemId)
{
	// Start of user code for method getAvailableNodeId:
	if(!subsystems.count(subsystemId) || !subsystems[subsystemId])
	{
		return Node::MINIMUM_ID;
	}

	for(int i = Node::MINIMUM_ID; i <= Node::MAXIMUM_ID; ++i)
	{
		if(!subsystems[subsystemId]->getNodes().count(i))
		{
			return static_cast<unsigned char>(i);
		}
	}

	THROW_EXCEPTION("No node IDs available on local subsystem");
	// End of user code
}


short SystemTree::getAvailableSubsystemId()
{
	// Start of user code for method getAvailableSubsystemId:
	for(int i = Subsystem::MINIMUM_ID; i <= Subsystem::MAXIMUM_ID; ++i)
	{
		if(!subsystems.count(i))
		{
			return static_cast<short>(i);
		}
	}

	THROW_EXCEPTION("No Subsystem IDs available on system");
	// End of user code
}


bool SystemTree::setComponentIdentification(const transport::Address &address, std::string name)
{
	// Start of user code for method setComponentIdentification:
	Component* cmpt = findComponent(address.getSubsystem(), address.getNode(), address.getComponent());
	if(!cmpt)
	{
		THROW_EXCEPTION("Attempted to set identification: " << name <<
						", to component address: " << address.toString() <<
						", which does not exist in system tree");
	}

	cmpt->setName(name);

	return true;
	// End of user code
}


bool SystemTree::setNodeIdentification(const transport::Address &address, std::string name)
{
	// Start of user code for method setNodeIdentification:
	Node* node = findNode(address.getSubsystem(), address.getNode());
	if(!node)
	{
		THROW_EXCEPTION("Attempted to set identification: " << name <<
						", to node address: " << address.toString() <<
						", which does not exist in system tree");
	}

	node->setName(name);

	return true;
	// End of user code
}


bool SystemTree::setSubsystemIdentification(const transport::Address &address, std::string name)
{
	// Start of user code for method setSubsystemIdentification:
	short subsystemId = address.getSubsystem();

	if(!subsystems.count(subsystemId))
	{
		THROW_EXCEPTION("Attempted to set identification: " << name <<
						", to subsystem address: " << address.toString() <<
						", which does not exist in system tree");
	}

	subsystems[subsystemId]->setName(name);

	return true;
	// End of user code
}


SystemTree& SystemTree::instance()
{
	// Start of user code for method instance:
	static SystemTree inst;
	return inst;
	// End of user code
}


bool SystemTree::addLocalComponent(const transport::Address &address)
{
	// Start of user code for method addLocalComponent:
	LOG_DEBUG("Adding Local Component: " << address);
	setThisSubsytem(address.getSubsystem());
	setThisNode(address.getNode());
	LOG_DEBUG("Adding Local Component: " << address);
	return addComponent(address);
	// End of user code
}




std::string SystemTree::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "System Tree:\n";
	for(std::map<short, Subsystem *>::const_iterator i = subsystems.begin();
		i != subsystems.end();
		++i)
	{
		oss << i->second->toString();
	}
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const SystemTree& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
Json::Value SystemTree::toJson() const
{
	Json::Value root;
	root["subsystems"] = Json::Value(Json::arrayValue);
	for(std::map<short, Subsystem *>::const_iterator i = subsystems.begin();
		i != subsystems.end();
		++i)
	{
		root["subsystems"].append(i->second->toJson());
	}
	return root;
}

std::string SystemTree::toXml(unsigned char level) const
{
	std::ostringstream prefix;
	for(unsigned char i = 0; i < level; i++)
	{
		prefix << "\t";
	}

	std::ostringstream oss;
	oss << prefix.str() << "<SystemTree>\n";
	for(std::map<short, Subsystem *>::const_iterator i = subsystems.begin();
		i != subsystems.end();
		++i)
	{
		oss << i->second->toXml(level+1);
	}
	oss << prefix.str() << "</SystemTree>\n";
	return oss.str();
}



Node* SystemTree::findNode(short subsId, unsigned char nodeId)
{
	if(!subsystems.count(subsId))
	{
		return NULL;
	}

	std::map<unsigned char, Node *>& nodes = const_cast<std::map<unsigned char, Node *>&>(subsystems[subsId]->getNodes());

	if(!nodes.count(nodeId))
	{
		return NULL;
	}

	return nodes[nodeId];
}

Component* SystemTree::findComponent(short subsId, unsigned char nodeId, unsigned char cmptId)
{
	Node *node = findNode(subsId, nodeId);
	if(!node)
	{
		return NULL;
	}

	std::map<unsigned char, Component *>& cmptMap = const_cast<std::map<unsigned char, Component *>&>(node->getComponents());

	if(!cmptMap.count(cmptId))
	{
		return NULL;
	}

	return cmptMap[cmptId];
}

std::vector<uint16_t> SystemTree::getSubsystemIds()
{
	std::vector<uint16_t> ids;
	for(std::map<short, Subsystem*>::iterator i = subsystems.begin();
		i != subsystems.end();
		++i)
	{
		ids.push_back(i->first);
	}

	return ids;
}

std::vector<transport::Address> SystemTree::lookupService(std::string uri, int versionMajor, int versionMinor, transport::Address addressHint)
{
	std::vector<transport::Address> addresses;

	std::map<short, Subsystem*>::iterator i;
	if(addressHint.getSubsystem() == transport::Address::ANY_SUBSYSTEM)
	{
		i = subsystems.begin();
		while(i != subsystems.end())
		{
			std::vector<transport::Address> additions = i->second->lookupService(uri, versionMajor, versionMinor, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
			++i;
		}
	}
	else
	{
		i = subsystems.find(addressHint.getSubsystem());
		if(i != subsystems.end())
		{
			std::vector<transport::Address> additions = i->second->lookupService(uri, versionMajor, versionMinor, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
		}
	}

	return addresses;
}

std::vector<transport::Address> SystemTree::lookupComponent(std::string name, transport::Address addressHint)
{
	std::vector<transport::Address> addresses;

	std::map<short, Subsystem*>::iterator i;
	if(addressHint.getSubsystem() == transport::Address::ANY_SUBSYSTEM)
	{
		i = subsystems.begin();
		while(i != subsystems.end())
		{
			std::vector<transport::Address> additions = i->second->lookupComponent(name, addressHint);
			addresses.insert(addresses.begin(), additions.begin(), additions.end());
			++i;
		}
	}
	else
	{
		i = subsystems.find(addressHint.getSubsystem());
		if(i != subsystems.end())
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

