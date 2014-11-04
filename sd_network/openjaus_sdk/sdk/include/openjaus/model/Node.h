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
#ifndef MODEL_NODE_H
#define MODEL_NODE_H

#include "openjaus/model/Component.h"
#include "openjaus/model/Subsystem.h"
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
class Subsystem;

/// \class Node Node.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Node 
{
public:
	Node(); 
	virtual ~Node();
	// Start of user code for additional constructors
	Node(const Node &copy);
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

	/// Accessor to get the value of components.
	const std::map< unsigned char, Component * >& getComponents() const;


	/// Accessor to get the value of subsystem.
	Subsystem* getSubsystem() const;

	/// Accessor to set value of subsystem.
	/// \param subsystem The value of the new subsystem.
	bool setSubsystem(Subsystem* subsystem);

	/// Operation addComponent.
	/// \param component 
	 bool addComponent(Component *component);

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Node& object);

protected:
	// Member attributes & references
	std::string name;
	int id;
	std::map< unsigned char, Component * > components;
	Subsystem *subsystem;

// Start of user code for additional member data
public:
	static const unsigned char MINIMUM_ID = 1;
	static const unsigned char MAXIMUM_ID = 254;
	Json::Value toJson() const;
	std::string toXml(unsigned char level=0) const;
	std::vector<transport::Address> lookupService(std::string uri, int versionMajor, int versionMinor, transport::Address& addressHint);
	std::vector<transport::Address> lookupComponent(std::string name, transport::Address& addressHint);
// End of user code

}; // class Node

// Start of user code for inline functions
// End of user code



} // namespace model
} // namespace openjaus

#endif // MODEL_NODE_H

