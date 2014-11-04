/**
\file InetAddress.h

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

#include "openjaus/system/InetAddress.h"
#include <sstream>
// Start of user code for additional includes
#include "openjaus/system/Exception.h"

#ifdef WIN32
	#define JAUS_EXPORT	__declspec(dllexport)
	#include <winsock2.h>
	#include <ws2tcpip.h>
    #ifndef socklen_t
        typedef int socklen_t;
    #endif
	#define CLOSE_SOCKET closesocket
	typedef unsigned int size_t;
	#pragma comment(lib, "Ws2_32.lib")
#elif defined(__linux) || defined(linux) || defined(__linux__) || defined(__APPLE__) || defined(__QNX__)
	#include <unistd.h>
	#include <sys/socket.h>
	#include <netdb.h>
	#include <arpa/inet.h>
	#include <sys/time.h>
	#include <netinet/in.h>
	#include <sys/types.h>
	#define CLOSE_SOCKET close
	#define JAUS_EXPORT
#else
	#error "No Socket implementation defined for this platform."
#endif

#include <sstream>
#include <string.h>
// End of user code

namespace openjaus
{
namespace system
{

// Start of user code for default constructor:
InetAddress::InetAddress() :
		value(0)
{
}
// End of user code

// Start of user code for default destructor:
InetAddress::~InetAddress()
{
}
// End of user code

int InetAddress::getValue() const
{
	// Start of user code for accessor getValue:
	
	return value;
	// End of user code
}

bool InetAddress::setValue(int value)
{
	// Start of user code for accessor setValue:
	this->value = value;
	return true;
	// End of user code
}



// Class Methods
InetAddress InetAddress::getByName(std::string name)
{
	// Start of user code for method getByName:
#ifdef WIN32
	// Initialize the socket subsystem
	WSADATA wsaData;
	int err = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if(err != 0)
	{
		THROW_EXCEPTION("Could not get host: " << name);
	}
#endif

	struct hostent *host = gethostbyname(name.c_str());
	if(host == NULL)
	{
		THROW_EXCEPTION("Could not get host: " << name);
	}

	InetAddress address;
	address.setValue(*reinterpret_cast<int *>(host->h_addr_list[0]));
	return address;
	// End of user code
}


InetAddress InetAddress::getLocalHost()
{
	// Start of user code for method getLocalHost:
	return getByName("127.0.0.1");
	// End of user code
}


InetAddress InetAddress::anyAddress()
{
	// Start of user code for method anyAddress:
	InetAddress result;

	result.value = INADDR_ANY;

	return result;
	// End of user code
}


uint32_t InetAddress::hash()
{
	// Start of user code for method hash:
	return this->value;
	// End of user code
}




std::string InetAddress::toString() const
{	
	// Start of user code for toString
	struct in_addr inAddress;
	memset(&inAddress, 0, sizeof(inAddress));
	inAddress.s_addr = value;

	std::ostringstream oss;
	oss << inet_ntoa(inAddress);
	return oss.str();
	// End of user code
}

std::ostream& operator<<(std::ostream& output, const InetAddress& object)
{
    output << object.toString();
    return output;
}
// Start of user code for additional methods
InetAddress::InetAddress(int value)
{
	this->value = value;
}
// End of user code

} // namespace system
} // namespace openjaus

