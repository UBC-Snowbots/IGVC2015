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
#ifndef MODEL_SYSTEMTREE_H
#define MODEL_SYSTEMTREE_H

#include "openjaus/transport/Address.h"
#include "openjaus/model/Component.h"
#include "openjaus/model/Node.h"
#include "openjaus/model/Subsystem.h"
#include "openjaus/model/SystemTree.h"
#include "openjaus/system/Mutex.h"
#include <map>
#include <string>
#include "openjaus/types.h"
#include <ostream>

// Start of user code for additional includes
#include "json/json.h"
// End of user code

namespace openjaus
{
namespace model
{
class Component;
class Node;
class Subsystem;
class SystemTree;

/// \class SystemTree SystemTree.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT SystemTree 
{
private:
	SystemTree(); 
	virtual ~SystemTree();
	// Start of user code for additional constructors
	// End of user code

public:
	/// Accessor to get the value of subsystems.
	const std::map< short, Subsystem * >& getSubsystems() const;


	/// Accessor to get the value of thisSubsytem.
	uint16_t getThisSubsytem() const;

	/// Accessor to set value of thisSubsytem.
	/// \param thisSubsytem The value of the new thisSubsytem.
	bool setThisSubsytem(uint16_t thisSubsytem);

	/// Accessor to get the value of thisNode.
	uint8_t getThisNode() const;

	/// Accessor to set value of thisNode.
	/// \param thisNode The value of the new thisNode.
	bool setThisNode(uint8_t thisNode);

	/// Accessor to get the value of mutex.
	const system::Mutex& getMutex() const;



	/// \param address 
	inline bool hasComponent(const transport::Address &address);

	/// Operation hasComponent.
	/// \param subsystemId 
	/// \param nodeId 
	/// \param componentId 
	 bool hasComponent(short subsystemId, unsigned char nodeId, unsigned char componentId);


	/// \param address 
	inline bool hasNode(const transport::Address &address);

	/// Operation hasNode.
	/// \param subsystemId 
	/// \param nodeId 
	 bool hasNode(short subsystemId, unsigned char nodeId);


	/// \param address 
	inline bool hasSubsystem(const transport::Address &address);

	/// Operation hasSubsystem.
	/// \param subsystemId 
	 bool hasSubsystem(short subsystemId);

	/// Operation hasComponentIdentification.
	/// \param component 
	 bool hasComponentIdentification(Component &component);


	/// \param address 
	inline bool hasComponentIdentification(const transport::Address &address);

	/// Operation hasComponentIdentification.
	/// \param subsystemId 
	/// \param nodeId 
	/// \param componentId 
	 bool hasComponentIdentification(short subsystemId, unsigned char nodeId, unsigned char componentId);

	/// Operation hasNodeIdentification.
	/// \param node 
	 bool hasNodeIdentification(Node &node);


	/// \param address 
	inline bool hasNodeIdentification(const transport::Address &address);

	/// Operation hasNodeIdentification.
	/// \param subsystemId 
	/// \param nodeId 
	 bool hasNodeIdentification(short subsystemId, unsigned char nodeId);

	/// Operation hasSubsystemIdentification.
	/// \param subsystem 
	 bool hasSubsystemIdentification(Subsystem &subsystem);


	/// \param address 
	inline bool hasSubsystemIdentification(const transport::Address &address);

	/// Operation hasSubsystemIdentification.
	/// \param subsystemId 
	 bool hasSubsystemIdentification(short subsystemId);

	/// Operation hasComponentConfiguration.
	/// \param component 
	 bool hasComponentConfiguration(Component &component);


	/// \param address 
	inline bool hasComponentConfiguration(const transport::Address &address);

	/// Operation hasComponentConfiguration.
	/// \param subsystemId 
	/// \param nodeId 
	/// \param componentId 
	 bool hasComponentConfiguration(short subsystemId, unsigned char nodeId, unsigned char componentId);

	/// Operation hasNodeConfiguration.
	/// \param node 
	 bool hasNodeConfiguration(Node &node);


	/// \param address 
	inline bool hasNodeConfiguration(const transport::Address &address);

	/// Operation hasNodeConfiguration.
	/// \param subsystemId 
	/// \param nodeId 
	 bool hasNodeConfiguration(short subsystemId, unsigned char nodeId);

	/// Operation hasSubsystemConfiguration.
	/// \param subsystem 
	 bool hasSubsystemConfiguration(Subsystem &subsystem);


	/// \param address 
	inline bool hasSubsystemConfiguration(const transport::Address &address);

	/// Operation hasSubsystemConfiguration.
	/// \param subsystemId 
	 bool hasSubsystemConfiguration(short subsystemId);

	/// Operation getComponent.
	/// \param component 
	 Component getComponent(Component &component);

	/// Operation getComponent.
	/// \param address 
	 Component getComponent(const transport::Address &address);

	/// Operation getComponent.
	/// \param subsystemId 
	/// \param nodeId 
	/// \param componentId 
	 Component getComponent(short subsystemId, unsigned char nodeId, unsigned char componentId);

	/// Operation getNode.
	/// \param node 
	 Node getNode(Node &node);

	/// Operation getNode.
	/// \param address 
	 Node getNode(const transport::Address &address);

	/// Operation getNode.
	/// \param subsystemId 
	/// \param nodeId 
	 Node getNode(short subsystemId, unsigned char nodeId);

	/// Operation getSubsystem.
	/// \param subsystem 
	 Subsystem getSubsystem(Subsystem &subsystem);

	/// Operation getSubsystem.
	/// \param address 
	 Subsystem getSubsystem(const transport::Address &address);

	/// Operation getSubsystem.
	/// \param subsystemId 
	 Subsystem getSubsystem(short subsystemId);

	/// Operation addService.
	/// \param address 
	/// \param uri 
	/// \param versionMajor 
	/// \param versionMinor 
	 bool addService(transport::Address address, std::string uri, int versionMajor, int versionMinor);

	/// Operation addComponent.
	/// \param component 
	 bool addComponent(Component &component);


	/// \param address 
	inline bool addComponent(const transport::Address &address);

	/// Operation addComponent.
	/// \param subsystemId 
	/// \param nodeId 
	/// \param componentId 
	 bool addComponent(short subsystemId, unsigned char nodeId, unsigned char componentId);

	/// Operation addNode.
	/// \param node 
	 bool addNode(Node &node);


	/// \param address 
	inline bool addNode(const transport::Address &address);

	/// Operation addNode.
	/// \param subsystemId 
	/// \param nodeId 
	 bool addNode(short subsystemId, unsigned char nodeId);

	/// Operation addSubsystem.
	/// \param subsystem 
	 bool addSubsystem(Subsystem &subsystem);


	/// \param address 
	inline bool addSubsystem(const transport::Address &address);

	/// Operation addSubsystem.
	/// \param subsystemId 
	 bool addSubsystem(short subsystemId);

	/// Operation replaceComponent.
	/// \param component 
	 bool replaceComponent(Component &component);

	/// Operation removeComponent.
	/// \param component 
	 bool removeComponent(Component &component);


	/// \param address 
	inline bool removeComponent(const transport::Address &address);

	/// Operation removeComponent.
	/// \param subsystemId 
	/// \param nodeId 
	/// \param componentId 
	 bool removeComponent(short subsystemId, unsigned char nodeId, unsigned char componentId);

	/// Operation removeNode.
	/// \param node 
	 bool removeNode(Node &node);


	/// \param address 
	inline bool removeNode(const transport::Address &address);

	/// Operation removeNode.
	/// \param subsystemId 
	/// \param nodeId 
	 bool removeNode(short subsystemId, unsigned char nodeId);

	/// Operation removeSubsystem.
	/// \param subsystem 
	 bool removeSubsystem(Subsystem &subsystem);


	/// \param address 
	inline bool removeSubsystem(const transport::Address &address);

	/// Operation removeSubsystem.
	/// \param subsystemId 
	 bool removeSubsystem(short subsystemId);

	/// Operation lock.
	 void lock();

	/// Operation unlock.
	 void unlock();

	/// Operation getAvailableComponentId.
	/// \param subsystemId 
	/// \param nodeId 
	 unsigned char getAvailableComponentId(short subsystemId, unsigned char nodeId);

	/// Operation getAvailableNodeId.
	/// \param subsystemId 
	 unsigned char getAvailableNodeId(short subsystemId);

	/// Operation getAvailableSubsystemId.
	 short getAvailableSubsystemId();

	/// Operation setComponentIdentification.
	/// \param address 
	/// \param name 
	 bool setComponentIdentification(const transport::Address &address, std::string name);

	/// Operation setNodeIdentification.
	/// \param address 
	/// \param name 
	 bool setNodeIdentification(const transport::Address &address, std::string name);

	/// Operation setSubsystemIdentification.
	/// \param address 
	/// \param name 
	 bool setSubsystemIdentification(const transport::Address &address, std::string name);


	static SystemTree& instance();

	/// Operation addLocalComponent.
	/// \param address 
	 bool addLocalComponent(const transport::Address &address);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const SystemTree& object);

protected:
	// Member attributes & references
	std::map< short, Subsystem * > subsystems;
	uint16_t thisSubsytem;
	uint8_t thisNode;
	system::Mutex mutex;

// Start of user code for additional member data
	Node* findNode(short subsId, unsigned char nodeId);
	Component* findComponent(short subsId, unsigned char nodeId, unsigned char cmptId);

public:
	std::vector<uint16_t> getSubsystemIds();
	Json::Value toJson() const;
	std::string toXml(unsigned char level=0) const;

	std::vector<transport::Address> lookupService(	std::string uri,
													int versionMajor = Service::ANY_VERSION,
													int versionMinor = Service::ANY_VERSION,
													transport::Address addressHint = transport::Address(transport::Address::ANY_SUBSYSTEM, transport::Address::ANY_NODE,transport::Address::ANY_COMPONENT) );

	std::vector<transport::Address> lookupComponent(std::string name,
													transport::Address addressHint = transport::Address(transport::Address::ANY_SUBSYSTEM, transport::Address::ANY_NODE,transport::Address::ANY_COMPONENT) );

// End of user code

}; // class SystemTree

// Start of user code for inline functions
// End of user code

inline bool SystemTree::hasComponent(const transport::Address &address)
{
	// Start of user code for method hasComponent(const transport::Address &address):
	return hasComponent(address.getSubsystem(), address.getNode(), address.getComponent());;
	// End of user code
}

inline bool SystemTree::hasNode(const transport::Address &address)
{
	// Start of user code for method hasNode(const transport::Address &address):
	return hasNode(address.getSubsystem(), address.getNode());;
	// End of user code
}

inline bool SystemTree::hasSubsystem(const transport::Address &address)
{
	// Start of user code for method hasSubsystem(const transport::Address &address):
	return hasSubsystem(address.getSubsystem());;
	// End of user code
}

inline bool SystemTree::hasComponentIdentification(const transport::Address &address)
{
	// Start of user code for method hasComponentIdentification(const transport::Address &address):
	return hasComponentIdentification(address.getSubsystem(), address.getNode(), address.getComponent());;
	// End of user code
}

inline bool SystemTree::hasNodeIdentification(const transport::Address &address)
{
	// Start of user code for method hasNodeIdentification(const transport::Address &address):
	return hasNodeIdentification(address.getSubsystem(), address.getNode());;
	// End of user code
}

inline bool SystemTree::hasSubsystemIdentification(const transport::Address &address)
{
	// Start of user code for method hasSubsystemIdentification(const transport::Address &address):
	return hasSubsystemIdentification(address.getSubsystem());;
	// End of user code
}

inline bool SystemTree::hasComponentConfiguration(const transport::Address &address)
{
	// Start of user code for method hasComponentConfiguration(const transport::Address &address):
	return hasComponentConfiguration(address.getSubsystem(), address.getNode(), address.getComponent());;
	// End of user code
}

inline bool SystemTree::hasNodeConfiguration(const transport::Address &address)
{
	// Start of user code for method hasNodeConfiguration(const transport::Address &address):
	return hasNodeConfiguration(address.getSubsystem(), address.getNode());;
	// End of user code
}

inline bool SystemTree::hasSubsystemConfiguration(const transport::Address &address)
{
	// Start of user code for method hasSubsystemConfiguration(const transport::Address &address):
	return hasSubsystemConfiguration(address.getSubsystem());;
	// End of user code
}

inline bool SystemTree::addComponent(const transport::Address &address)
{
	// Start of user code for method addComponent(const transport::Address &address):
	return addComponent(address.getSubsystem(), address.getNode(), address.getComponent());;
	// End of user code
}

inline bool SystemTree::addNode(const transport::Address &address)
{
	// Start of user code for method addNode(const transport::Address &address):
	return addNode(address.getSubsystem(), address.getNode());;
	// End of user code
}

inline bool SystemTree::addSubsystem(const transport::Address &address)
{
	// Start of user code for method addSubsystem(const transport::Address &address):
	return addSubsystem(address.getSubsystem());;
	// End of user code
}

inline bool SystemTree::removeComponent(const transport::Address &address)
{
	// Start of user code for method removeComponent(const transport::Address &address):
	return removeComponent(address.getSubsystem(), address.getNode(), address.getComponent());;
	// End of user code
}

inline bool SystemTree::removeNode(const transport::Address &address)
{
	// Start of user code for method removeNode(const transport::Address &address):
	return removeNode(address.getSubsystem(), address.getNode());;
	// End of user code
}

inline bool SystemTree::removeSubsystem(const transport::Address &address)
{
	// Start of user code for method removeSubsystem(const transport::Address &address):
	return removeSubsystem(address.getSubsystem());;
	// End of user code
}

} // namespace model
} // namespace openjaus

#endif // MODEL_SYSTEMTREE_H

