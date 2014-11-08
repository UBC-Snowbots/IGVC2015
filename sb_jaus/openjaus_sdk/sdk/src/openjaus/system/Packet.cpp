/**
\file Packet.h

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

#include "openjaus/system/Packet.h"
#include <sstream>
// Start of user code for additional includes
#include <stdlib.h>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
Packet::Packet() :
		port(0),
		address()
{
}
// End of user code

// Start of user code for default destructor:
Packet::~Packet()
{
}
// End of user code

uint16_t Packet::getPort() const
{
	// Start of user code for accessor getPort:
	
	return port;
	// End of user code
}

bool Packet::setPort(uint16_t port)
{
	// Start of user code for accessor setPort:
	this->port = port;
	
	return true;
	// End of user code
}


const InetAddress& Packet::getAddress() const
{
	// Start of user code for accessor getAddress:
	
	return address;
	// End of user code
}

bool Packet::setAddress(const InetAddress& address)
{
	// Start of user code for accessor setAddress:
	this->address = address;
	
	return true;
	// End of user code
}



// Class Methods
uint64_t Packet::addressHash()
{
	// Start of user code for method addressHash:
	return (address.hash() << 16) + port;
	// End of user code
}




std::string Packet::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "Packet: Address: " << address << ":" << static_cast<unsigned short>(port) << ", Size (" << const_cast<Packet *>(this)->containedBytes() << "/" << maxSize << ")";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const Packet& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
Packet::Packet(int size) :
		port(0),
		address()
{
	setMaxSize(size);
}

// End of user code

} // namespace system
} // namespace openjaus

