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
#ifndef MODEL_COMPONENT_H
#define MODEL_COMPONENT_H

#include "openjaus/model/Service.h"
#include <map>
#include "openjaus/transport/Address.h"
#include "openjaus/model/StateMachineRunner.h"
#include "openjaus/transport/Interface.h"
#include "openjaus/model/Node.h"
#include "openjaus/model/SystemTree.h"
#include "openjaus/model/Component.h"
#include <vector>
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
class Service;
class StateMachineRunner;
class Node;
class SystemTree;
class Component;

/// \class Component Component.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Component 
{
public:
	Component(); 
	virtual ~Component();
	// Start of user code for additional constructors
	Component(const Component& copy);
	// End of user code
	/// Accessor to get the value of name.
	std::string getName() const;

	/// Accessor to set value of name.
	/// \param name The value of the new name.
	bool setName(std::string name);

	/// Accessor to get the value of id.
	int getId() const;

	/// Accessor to set value of id.
	/// \param id The value of the new id.
	bool setId(int id);

	/// Accessor to get the value of services.
	const std::map< std::string, Service * >& getServices() const;


	/// Accessor to get the value of authority.
	uint8_t getAuthority() const;

	/// Accessor to set value of authority.
	/// \param authority The value of the new authority.
	bool setAuthority(uint8_t authority);

	/// Accessor to get the value of address.
	const transport::Address& getAddress() const;

	/// Accessor to set value of address.
	/// \param address The value of the new address.
	bool setAddress(const transport::Address& address);

	/// Accessor to get the value of runners.
	const std::vector< StateMachineRunner* >& getRunners() const;

	/// Accessor to set value of runners.
	/// \param runners The value of the new runners.
	bool setRunners(const StateMachineRunner& runners);

	/// Accessor to get the value of interfaces.
	const std::vector< transport::Interface* >& getInterfaces() const;

	/// Accessor to set value of interfaces.
	/// \param interfaces The value of the new interfaces.
	bool setInterfaces(const transport::Interface& interfaces);

	/// Accessor to get the value of node.
	Node* getNode() const;

	/// Accessor to set value of node.
	/// \param node The value of the new node.
	bool setNode(Node* node);

	/// Accessor to get the value of systemTree.
	SystemTree* getSystemTree() const;

	/// Accessor to set value of systemTree.
	/// \param systemTree The value of the new systemTree.
	bool setSystemTree(SystemTree* systemTree);

	/// Accessor to get the value of implements.
	std::vector< Service* >* getImplements() const;

	/// Accessor to set value of implements.
	/// \param implements The value of the new implements.
	bool setImplements(Service* implements);

	/// Accessor to get the value of inheritsFrom.
	Component* getInheritsFrom() const;

	/// Accessor to set value of inheritsFrom.
	/// \param inheritsFrom The value of the new inheritsFrom.
	bool setInheritsFrom(Component* inheritsFrom);

	/// Operation addService.
	/// \param service 
	 bool addService(Service *service);

	/// Operation run.
	 void run();

	/// Operation stop.
	 void stop();

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Component& object);

protected:
	// Member attributes & references
	std::string name;
	int id;
	std::map< std::string, Service * > services;
	uint8_t authority;
	transport::Address address;
	std::vector< StateMachineRunner* > runners;
	std::vector< transport::Interface* > interfaces;
	Node *node;
	SystemTree *systemTree;
	std::vector< Service* > *implements;
	Component *inheritsFrom;

// Start of user code for additional member data
public:
	void clearServices();
	Json::Value toJson() const;
	std::string toXml(unsigned char level=0) const;

	static const unsigned char MINIMUM_ID = 1;
	static const unsigned char MAXIMUM_ID = 254;
	static const uint8_t DEFAULT_AUTHORITY = 127;

	bool containsService(std::string uri, int versionMajor = Service::ANY_VERSION, int versionMinor = Service::ANY_VERSION);
// End of user code

}; // class Component

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_COMPONENT_H

