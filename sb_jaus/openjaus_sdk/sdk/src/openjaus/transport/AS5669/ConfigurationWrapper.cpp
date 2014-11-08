/**
\file ConfigurationWrapper.h

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

#include "openjaus/transport/AS5669/ConfigurationWrapper.h"
#include <sstream>
// Start of user code for additional includes
// End of user code

namespace openjaus
{
namespace transport
{
namespace AS5669
{

// Start of user code for default constructor:
ConfigurationWrapper::ConfigurationWrapper()
{
	setType(CONFIGURATION);
}
// End of user code

// Start of user code for default destructor:
ConfigurationWrapper::~ConfigurationWrapper()
{
}
// End of user code

uint16_t ConfigurationWrapper::getPort() const
{
	// Start of user code for accessor getPort:
	
	return port;
	// End of user code
}

bool ConfigurationWrapper::setPort(uint16_t port)
{
	// Start of user code for accessor setPort:
	this->port = port;
	return true;
	// End of user code
}


const system::InetAddress& ConfigurationWrapper::getIpAddress() const
{
	// Start of user code for accessor getIpAddress:
	
	return ipAddress;
	// End of user code
}

bool ConfigurationWrapper::setIpAddress(const system::InetAddress& ipAddress)
{
	// Start of user code for accessor setIpAddress:
	this->ipAddress = ipAddress;
	return true;
	// End of user code
}



// Class Methods
int ConfigurationWrapper::to(system::Buffer *dst)
{
	// Start of user code for method to:
	int initialBufferSize = dst->remainingBytes();

	unsigned char tempByte(0);
	tempByte |= ((type & 0x3F) << 2);
	dst->pack(tempByte);

	short length = static_cast<short>(payload->length());
	dst->pack(length);

	payload->to(dst);

	return initialBufferSize - dst->remainingBytes();
	// End of user code
}


int ConfigurationWrapper::from(system::Buffer *src)
{
	// Start of user code for method from:
	int initialBufferSize = src->remainingBytes();

	unsigned char tempByte = 0;
	src->unpack(tempByte);
	type = static_cast<WrapperType>((tempByte >> 2) & 0x3F);

	short length;
	src->unpack(length);

	if(!payload)
	{
		payload = new system::Buffer();
	}

	if(length)
	{
		dynamic_cast<system::Buffer *>(payload)->setMaxSize(length);
		std::memcpy(dynamic_cast<system::Buffer *>(payload)->getBuffer(), src->getPointer(), length);
//		pointer += length;
//		bufferSize -= length;
	}

	return initialBufferSize - src->remainingBytes();
	// End of user code
}


int ConfigurationWrapper::length()
{
	// Start of user code for method length:
	int result = 0;

	return result;
	// End of user code
}




std::string ConfigurationWrapper::toString() const
{	
	// Start of user code for toString
	std::ostringstream oss;
	oss << "";
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const ConfigurationWrapper& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
// End of user code

} // namespace AS5669
} // namespace transport
} // namespace openjaus

