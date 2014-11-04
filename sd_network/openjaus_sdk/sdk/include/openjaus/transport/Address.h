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
#ifndef TRANSPORT_ADDRESS_H
#define TRANSPORT_ADDRESS_H

#include "openjaus/system/Transportable.h"
#include <string>
#include "openjaus/types.h"
#include <ostream>

#include "openjaus/system/Buffer.h"
// Start of user code for additional includes
#include "json/json.h"
// End of user code

namespace openjaus
{
namespace transport
{

/// \class Address Address.h
/// \brief This is a brief description.
/// Detailed description.
/// \author Name (name@email.com)
class OPENJAUS_EXPORT Address : public system::Transportable
{
public:
	Address(); 
	virtual ~Address();
	// Start of user code for additional constructors
	Address(uint16_t subsystem, unsigned char node, unsigned char component);
	// End of user code
	/// Accessor to get the value of subsystem.
	uint16_t getSubsystem() const;

	/// Accessor to set value of subsystem.
	/// \param subsystem The value of the new subsystem.
	bool setSubsystem(uint16_t subsystem);

	/// Accessor to get the value of node.
	unsigned char getNode() const;

	/// Accessor to set value of node.
	/// \param node The value of the new node.
	bool setNode(unsigned char node);

	/// Accessor to get the value of component.
	unsigned char getComponent() const;

	/// Accessor to set value of component.
	/// \param component The value of the new component.
	bool setComponent(unsigned char component);

	/// Accessor to get the value of hash.
	int getHash() const;


	/// Operation isValid.
	 bool isValid();

	// Inherited pure virtuals from Transportable that need to be implemented
	virtual int to(system::Buffer *dst);	
	virtual int from(system::Buffer *src);	
	virtual int length();	

	std::string toString() const;
	OPENJAUS_EXPORT friend std::ostream& operator<<(std::ostream& output, const Address& object);

protected:
	// Member attributes & references
	uint16_t subsystem;
	unsigned char node;
	unsigned char component;

// Start of user code for additional member data
public:
	static const unsigned short ANY_SUBSYSTEM = 65535;
	static const unsigned char ANY_NODE = 255;
	static const unsigned char ANY_COMPONENT = 255;

	static const unsigned short THIS_SUBSYSTEM = 0;
	static const unsigned char THIS_NODE = 0;
	static const unsigned char THIS_COMPONENT = 0;

	Json::Value toJson() const;
	std::string toXml(unsigned char level=0) const;

	bool operator==(const Address &other) const;
	bool operator!=(const Address &other) const;
	bool operator<(const Address &other) const;
// End of user code

}; // class Address

// Start of user code for inline functions
// End of user code



} // namespace transport
} // namespace openjaus

#endif // TRANSPORT_ADDRESS_H

